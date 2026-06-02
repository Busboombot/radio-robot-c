"""Movie capture and encoding utilities for de-skewed playfield sessions.

Capture writes raw de-skewed playfield frames into a session directory.
Movie creation is a separate step that can optionally add overlays while
encoding.

Usage:
  uv run python -m robot_radio.movie list
  uv run python -m robot_radio.movie save-frames --camera 3 --duration 10
  uv run python -m robot_radio.movie make-movie
  uv run python -m robot_radio.movie make-movie --session cam3-20260328-120000
  uv run python -m robot_radio.movie make-movie --session data/recordings/movies/cam3-20260328-120000 --overlay-tags
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from aprilcam.camera.camutil import get_device_name
from aprilcam.calibration.calibration import load_calibration_for_camera

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_DATA_DIR = SCRIPT_DIR.parent / "data"
DEFAULT_MOVIE_ROOT = DEFAULT_DATA_DIR / "recordings" / "movies"
MANIFEST_NAME = "manifest.json"
RAW_DIR_NAME = "raw"
ANNOTATED_DIR_NAME = "annotated"
META_DIR_NAME = "meta"


@dataclass(frozen=True)
class PlayfieldCalibration:
    camera_name: str
    field_width_cm: float
    field_height_cm: float
    homography: np.ndarray
    camera_matrix: np.ndarray | None
    dist_coeffs: np.ndarray | None

    def warp_matrix(self, pixels_per_cm: float) -> np.ndarray:
        scale = np.array(
            [[pixels_per_cm, 0.0, 0.0], [0.0, pixels_per_cm, 0.0], [0.0, 0.0, 1.0]],
            dtype=float,
        )
        return scale @ self.homography

    def output_size(self, pixels_per_cm: float) -> tuple[int, int]:
        width_px = max(1, int(round(self.field_width_cm * pixels_per_cm)))
        height_px = max(1, int(round(self.field_height_cm * pixels_per_cm)))
        return width_px, height_px


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())


def _load_playfield_calibration(
    camera: int,
    data_dir: Path = DEFAULT_DATA_DIR,
) -> PlayfieldCalibration:
    calibration_path = data_dir / "calibration.json"
    if not calibration_path.exists():
        raise FileNotFoundError(f"Missing playfield calibration: {calibration_path}")

    payload = _load_json(calibration_path)
    if payload.get("type") != "playfield":
        raise RuntimeError(f"Unsupported calibration type in {calibration_path}")

    camera_name = get_device_name(camera)
    calibration = load_calibration_for_camera(camera_name, data_dir)
    if calibration is None:
        raise RuntimeError(
            f"No calibration entry for camera {camera} ({camera_name}) in {calibration_path}"
        )

    return PlayfieldCalibration(
        camera_name=camera_name,
        field_width_cm=float(payload["field_width_cm"]),
        field_height_cm=float(payload["field_height_cm"]),
        homography=np.array(calibration.homography, dtype=float),
        camera_matrix=(None if calibration.camera_matrix is None
                       else np.array(calibration.camera_matrix, dtype=float)),
        dist_coeffs=(None if calibration.dist_coeffs is None
                     else np.array(calibration.dist_coeffs, dtype=float)),
    )


def _ensure_session_dir(session_dir: Path) -> None:
    (session_dir / RAW_DIR_NAME).mkdir(parents=True, exist_ok=True)
    (session_dir / META_DIR_NAME).mkdir(parents=True, exist_ok=True)


def _frame_path(session_dir: Path, frame_index: int, suffix: str) -> Path:
    return session_dir / RAW_DIR_NAME / f"frame-{frame_index:06d}.{suffix}"


def _meta_path(session_dir: Path, frame_index: int) -> Path:
    return session_dir / META_DIR_NAME / f"frame-{frame_index:06d}.json"


def _manifest_path(session_dir: Path) -> Path:
    return session_dir / MANIFEST_NAME


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")


def _load_manifest(session_dir: Path) -> dict[str, Any]:
    manifest_path = _manifest_path(session_dir)
    if not manifest_path.exists():
        raise FileNotFoundError(f"Missing manifest: {manifest_path}")
    return _load_json(manifest_path)


def _resolve_session_dir(session: str | None, root_dir: Path = DEFAULT_MOVIE_ROOT) -> Path | None:
    if session is None:
        return None
    candidate = Path(session)
    if candidate.is_dir():
        return candidate.resolve()
    alt = (root_dir / session).resolve()
    if alt.is_dir():
        return alt
    raise FileNotFoundError(f"Movie session not found: {session}")


def list_movie_sessions(root_dir: str | Path = DEFAULT_MOVIE_ROOT) -> list[dict[str, Any]]:
    """
    List all available movie recording sessions.
    
    Scans the root directory for session directories containing valid manifest.json
    and raw/ subdirectories. Returns metadata for each session, suitable for display
    or for passing the session name to make_movie().
    
    Args:
        root_dir: Directory containing session subdirectories (default: data/recordings/movies).
    
    Returns:
        List of dicts, one per session, with keys:
            - name: Session directory name (use this for make_movie session parameter)
            - path: Full path to session directory
            - frames_saved: Number of frames captured
            - camera: Camera index (3 or 2)
            - camera_name: Device name (e.g., 'guvov')
            - created_at: ISO timestamp when recording started
            - movie_path: Path to output movie.mp4 if one exists, else None
        
        Returns empty list if root_dir doesn't exist or contains no valid sessions.
    
    Example:
        sessions = list_movie_sessions()
        for s in sessions:
            print(f"{s['name']}: {s['frames_saved']} frames ({s['camera_name']})")
        
        # Use a session name with make_movie
        if sessions:
            make_movie(session=sessions[0]['name'])
    """
    root = Path(root_dir)
    if not root.exists():
        return []

    sessions: list[dict[str, Any]] = []
    for child in sorted(root.iterdir()):
        if not child.is_dir():
            continue
        manifest_path = child / MANIFEST_NAME
        raw_dir = child / RAW_DIR_NAME
        if not manifest_path.exists() or not raw_dir.is_dir():
            continue
        try:
            manifest = _load_json(manifest_path)
        except Exception:
            manifest = {}
        sessions.append({
            "name": child.name,
            "path": str(child),
            "frames_saved": int(manifest.get("frames_saved", 0)),
            "camera": manifest.get("camera"),
            "camera_name": manifest.get("camera_name"),
            "created_at": manifest.get("created_at"),
            "movie_path": manifest.get("movie", {}).get("output_path"),
        })
    return sessions


def _preprocess_diff(frame_bgr: np.ndarray, size: tuple[int, int] = (160, 120)) -> np.ndarray:
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    return cv2.resize(gray, size, interpolation=cv2.INTER_AREA)


def _difference_score(current_small: np.ndarray, previous_small: np.ndarray | None) -> float:
    if previous_small is None:
        return 255.0
    diff = cv2.absdiff(current_small, previous_small)
    return float(diff.mean())


def _deskew_frame(frame_bgr: np.ndarray, calibration: PlayfieldCalibration,
                  pixels_per_cm: float) -> np.ndarray:
    if calibration.camera_matrix is not None and calibration.dist_coeffs is not None:
        frame_bgr = cv2.undistort(frame_bgr, calibration.camera_matrix, calibration.dist_coeffs)
    warp = calibration.warp_matrix(pixels_per_cm)
    out_w, out_h = calibration.output_size(pixels_per_cm)
    return cv2.warpPerspective(frame_bgr, warp, (out_w, out_h))


def save_movie_frames(
    camera: int = 3,
    duration_s: float = 0,
    max_frames: int = 0,
    root_dir: str | Path = DEFAULT_MOVIE_ROOT,
    session_name: str | None = None,
    pixels_per_cm: float = 8.0,
    diff_threshold: float = 2.0,
    min_interval_ms: int = 0,
    max_gap_s: float = 1.0,
    image_format: str = "jpg",
    jpeg_quality: int = 90,
) -> dict[str, Any]:
    """
    Capture de-skewed playfield frames during a task.
    
    Intended for agents to call during autonomous execution: frames are stored in a session
    directory with de-skewing applied, making them ready for direct analysis or movie encoding.
    
    Frames are only saved when motion is detected (exceeds diff_threshold) or when max_gap_s
    has elapsed since the last save. This balances recording detail with storage efficiency.
    
    Args:
        camera: Camera index to record from (3=B&W 88x72, 2=color 1920x1080). Default 3.
        duration_s: Seconds to record (0=infinite, until interrupted).
        max_frames: Max frames to capture (0=unlimited).
        root_dir: Base directory for sessions. Default: data/recordings/movies.
        session_name: Custom session directory name. If None, auto-generated as "cam{i}-{timestamp}".
        pixels_per_cm: Output resolution in calibrated playfield space (pixels per real-world cm).
                       8.0 = typical playfield ~40x30cm becomes ~320x240px per frame.
        diff_threshold: Motion sensitivity (0-255 scale). Frames saved if grayscale difference
                        from last saved frame >= this threshold. Default 2.0 (very sensitive).
        min_interval_ms: Min delay between consecutive saves after motion triggers. 0=no throttle.
        max_gap_s: Force-save at least every N seconds even if no motion (keeps long still scenes
                   from becoming completely empty). Default 1.0.
        image_format: 'jpg' (smaller) or 'png' (lossless). Default 'jpg'.
        jpeg_quality: JPEG quality 1-100 (ignored for PNG). Default 90.
    
    Returns:
        dict with keys:
            - session_dir: Full path to session directory (for reuse in make_movie)
            - frames_saved: Number of frames actually written
            - camera: Camera index used
            - camera_name: Device name (e.g., 'guvov')
            - frames_seen: Total frames examined
            - created_at: ISO timestamp of session creation
            - manifest_path: Path to manifest.json for reference
    
    Session Directory Structure:
        data/recordings/movies/cam3-20260402-120000/
          manifest.json          # metadata: session info, frame counts, paths
          raw/                   # de-skewed playfield frames
            frame-000000.jpg
            frame-000001.jpg
            ...
          meta/                  # per-frame metadata (timestamps, motion scores)
            frame-000000.json
            frame-000001.json
            ...
    
    Example for agents:
        result = save_movie_frames(camera=3, duration_s=30)
        session_name = result['session_dir']
        # Now run a task for 30s
        # Later, make a movie: make_movie(session=result['session_dir'])
    """
    if duration_s <= 0 and max_frames <= 0:
        raise ValueError("Specify duration_s > 0 or max_frames > 0")

    image_format = image_format.lower()
    if image_format not in {"jpg", "png"}:
        raise ValueError("image_format must be 'jpg' or 'png'")

    root = Path(root_dir)
    root.mkdir(parents=True, exist_ok=True)
    calibration = _load_playfield_calibration(camera)

    stamp = time.strftime("%Y%m%d-%H%M%S")
    session_slug = session_name or f"cam{camera}-{stamp}"
    session_dir = (root / session_slug).resolve()
    if session_dir.exists():
        raise FileExistsError(f"Movie session already exists: {session_dir}")
    _ensure_session_dir(session_dir)

    cap = cv2.VideoCapture(camera)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {camera}")

    frames_seen = 0
    frames_saved = 0
    previous_saved_small: np.ndarray | None = None
    last_saved_monotonic: float | None = None
    start_monotonic = time.monotonic()
    wall_clock_start = time.time()
    source_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    source_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    out_w, out_h = calibration.output_size(pixels_per_cm)

    manifest: dict[str, Any] = {
        "kind": "movie-session",
        "created_at": wall_clock_start,
        "camera": camera,
        "camera_name": calibration.camera_name,
        "field_width_cm": calibration.field_width_cm,
        "field_height_cm": calibration.field_height_cm,
        "pixels_per_cm": pixels_per_cm,
        "source_resolution": [source_width, source_height],
        "deskewed_resolution": [out_w, out_h],
        "diff_threshold": diff_threshold,
        "min_interval_ms": min_interval_ms,
        "max_gap_s": max_gap_s,
        "image_format": image_format,
        "jpeg_quality": jpeg_quality,
        "frames_seen": 0,
        "frames_saved": 0,
        "session_dir": str(session_dir),
        "raw_dir": str(session_dir / RAW_DIR_NAME),
    }
    _write_json(_manifest_path(session_dir), manifest)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frames_seen += 1
            elapsed = time.monotonic() - start_monotonic
            if duration_s > 0 and elapsed >= duration_s:
                break

            deskewed = _deskew_frame(frame, calibration, pixels_per_cm)
            current_small = _preprocess_diff(deskewed)
            diff_score = _difference_score(current_small, previous_saved_small)
            now = time.monotonic()

            save_due_to_gap = (
                last_saved_monotonic is None
                or max_gap_s <= 0
                or (now - last_saved_monotonic) >= max_gap_s
            )
            save_due_to_diff = diff_score >= diff_threshold
            respects_min_interval = (
                last_saved_monotonic is None
                or min_interval_ms <= 0
                or (now - last_saved_monotonic) * 1000.0 >= min_interval_ms
            )
            should_save = last_saved_monotonic is None or save_due_to_gap or (
                save_due_to_diff and respects_min_interval
            )

            if should_save:
                frame_path = _frame_path(session_dir, frames_saved, image_format)
                if image_format == "jpg":
                    ok = cv2.imwrite(
                        str(frame_path),
                        deskewed,
                        [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality],
                    )
                else:
                    ok = cv2.imwrite(str(frame_path), deskewed)
                if not ok:
                    raise RuntimeError(f"Failed to save frame: {frame_path}")

                frame_meta = {
                    "frame_index": frames_saved,
                    "frames_seen": frames_seen,
                    "timestamp": time.time(),
                    "elapsed_s": elapsed,
                    "diff_score": diff_score,
                    "path": str(frame_path),
                }
                _write_json(_meta_path(session_dir, frames_saved), frame_meta)
                frames_saved += 1
                previous_saved_small = current_small
                last_saved_monotonic = now

            if max_frames > 0 and frames_saved >= max_frames:
                break

    finally:
        cap.release()

    manifest.update({
        "completed_at": time.time(),
        "frames_seen": frames_seen,
        "frames_saved": frames_saved,
        "capture_duration_s": time.monotonic() - start_monotonic,
        "save_ratio": (frames_saved / frames_seen) if frames_seen else 0.0,
    })
    _write_json(_manifest_path(session_dir), manifest)
    return manifest


def _iter_frame_paths(session_dir: Path, use_annotated: bool = False) -> list[Path]:
    subdir = ANNOTATED_DIR_NAME if use_annotated else RAW_DIR_NAME
    base = session_dir / subdir
    if not base.is_dir():
        return []
    return sorted(
        [path for path in base.iterdir() if path.suffix.lower() in {".jpg", ".jpeg", ".png"}]
    )


def _build_tag_detector() -> cv2.aruco.ArucoDetector:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(dictionary, params)


def _overlay_tags(frame_bgr: np.ndarray, detector: cv2.aruco.ArucoDetector) -> np.ndarray:
    annotated = frame_bgr.copy()
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return annotated

    ids_flat = ids.flatten().tolist()
    cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
    for marker_corners, tag_id in zip(corners, ids_flat):
        pts = marker_corners[0]
        center = pts.mean(axis=0)
        cx, cy = int(center[0]), int(center[1])
        cv2.putText(
            annotated,
            str(tag_id),
            (cx + 6, cy - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return annotated


def _overlay_frame_label(frame_bgr: np.ndarray, label: str) -> np.ndarray:
    annotated = frame_bgr.copy()
    cv2.putText(
        annotated,
        label,
        (12, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        annotated,
        label,
        (12, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 0, 0),
        1,
        cv2.LINE_AA,
    )
    return annotated


def make_movie(
    session: str | None = None,
    root_dir: str | Path = DEFAULT_MOVIE_ROOT,
    output_path: str | None = None,
    fps: float = 15.0,
    codec: str = "mp4v",
    use_annotated: bool = False,
    overlay_tags: bool = False,
    overlay_frame_index: bool = False,
) -> dict[str, Any] | list[dict[str, Any]]:
    """
    Encode saved movie frames into an MP4 video file.
    
    Reads frames from a session directory (created by save_movie_frames), optionally applies
    overlays (AprilTag detection, frame indices), and encodes into MP4. If no session is
    specified, returns the list of available sessions instead.
    
    This is the second half of the movie workflow: save_movie_frames captures frames,
    make_movie encodes them into a playable video.
    
    Args:
        session: Session directory name (from save_movie_frames result) or full path.
                 If None, lists all available sessions and returns without encoding.
        root_dir: Base directory for sessions (default: data/recordings/movies).
        output_path: Output MP4 file path. If None, uses <session_dir>/movie.mp4.
        fps: Frames per second for output video (default 15.0).
        codec: FourCC codec string (default 'mp4v', most compatible). Other options:
               'H264' (hardware accelerated on some platforms, better compression),
               'MJPG' (motion JPEG, older compatibility),
               'MPEG' (MPEG-4 codec).
        use_annotated: If True, read from annotated/ subdirectory instead of raw/.
                       Requires pre-processing via annotate_movie_frames. Default False.
        overlay_tags: If True, detect AprilTags during encoding and draw bounding boxes.
                      Slows encoding significantly (per-frame detection). Default False.
        overlay_frame_index: If True, draw frame number in top-left corner. Default False.
    
    Returns:
        dict with encoding details (if session given):
            - message: "Movie encoded successfully"
            - output_path: Full path to output MP4
            - fps: Frames per second used
            - codec: Codec used
            - frames: Number of frames encoded
            - duration_s: Total duration in seconds (frames / fps)
            - overlays_used: List of overlays applied (e.g., ['tags', 'frame_index'])
        
        list of available sessions (if session is None):
            - Same format as list_movie_sessions output
    
    Example for agents:
        # First, capture frames during a task
        result = save_movie_frames(camera=3, duration_s=30)
        
        # Later, encode the session into a video
        movie_info = make_movie(
            session=result['session_dir'],
            fps=30,
            overlay_tags=True,
            overlay_frame_index=True
        )
        print(f"Movie saved to: {movie_info['output_path']}")
        
        # Or list available sessions
        sessions = make_movie()
        for s in sessions:
            print(f"{s['name']}: {s['frames_saved']} frames")
    """
    session_dir = _resolve_session_dir(session, Path(root_dir))
    if session_dir is None:
        return list_movie_sessions(root_dir)

    manifest = _load_manifest(session_dir)
    frame_paths = _iter_frame_paths(session_dir, use_annotated=use_annotated)
    if not frame_paths:
        raise RuntimeError(f"No frames found in session: {session_dir}")

    first_frame = cv2.imread(str(frame_paths[0]))
    if first_frame is None:
        raise RuntimeError(f"Cannot read frame: {frame_paths[0]}")

    height, width = first_frame.shape[:2]
    output = Path(output_path).resolve() if output_path else session_dir / "movie.mp4"
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output),
        cv2.VideoWriter_fourcc(*codec),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        raise RuntimeError(f"Cannot create movie writer for {output}")

    detector = _build_tag_detector() if overlay_tags else None

    try:
        for frame_number, frame_path in enumerate(frame_paths):
            frame = cv2.imread(str(frame_path))
            if frame is None:
                raise RuntimeError(f"Cannot read frame: {frame_path}")
            if detector is not None:
                frame = _overlay_tags(frame, detector)
            if overlay_frame_index:
                frame = _overlay_frame_label(frame, f"frame {frame_number:06d}")
            writer.write(frame)
    finally:
        writer.release()

    movie_info = {
        "output_path": str(output),
        "fps": fps,
        "codec": codec,
        "frames": len(frame_paths),
        "duration_s": len(frame_paths) / fps if fps > 0 else 0.0,
        "use_annotated": use_annotated,
        "overlay_tags": overlay_tags,
        "overlay_frame_index": overlay_frame_index,
    }
    manifest["movie"] = movie_info
    _write_json(_manifest_path(session_dir), manifest)
    return {
        "session_dir": str(session_dir),
        "session_name": session_dir.name,
        **movie_info,
    }


def _print_sessions(sessions: list[dict[str, Any]]) -> int:
    if not sessions:
        print("No movie frame sessions found.")
        return 0
    for session in sessions:
        print(
            f"{session['name']}: frames={session['frames_saved']} camera={session.get('camera')} "
            f"camera_name={session.get('camera_name')} path={session['path']}"
        )
    return 0


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Capture de-skewed playfield frames and build movies from saved sessions."
    )
    subparsers = parser.add_subparsers(dest="command")

    list_parser = subparsers.add_parser("list", help="List available movie frame sessions")
    list_parser.add_argument("--root-dir", type=str, default=str(DEFAULT_MOVIE_ROOT))

    save_parser = subparsers.add_parser("save-frames", help="Save raw de-skewed playfield frames")
    save_parser.add_argument("--camera", "-c", type=int, default=3)
    save_parser.add_argument("--duration", type=float, default=10.0)
    save_parser.add_argument("--max-frames", type=int, default=0)
    save_parser.add_argument("--root-dir", type=str, default=str(DEFAULT_MOVIE_ROOT))
    save_parser.add_argument("--session-name", type=str, default=None)
    save_parser.add_argument("--pixels-per-cm", type=float, default=8.0)
    save_parser.add_argument("--diff-threshold", type=float, default=2.0)
    save_parser.add_argument("--min-interval-ms", type=int, default=0)
    save_parser.add_argument("--max-gap-s", type=float, default=1.0)
    save_parser.add_argument("--image-format", type=str, default="jpg")
    save_parser.add_argument("--jpeg-quality", type=int, default=90)

    movie_parser = subparsers.add_parser(
        "make-movie",
        help="Build a movie from a saved frame session, or list sessions if none is specified",
    )
    movie_parser.add_argument("--session", type=str, default=None)
    movie_parser.add_argument("--root-dir", type=str, default=str(DEFAULT_MOVIE_ROOT))
    movie_parser.add_argument("--output", "-o", type=str, default=None)
    movie_parser.add_argument("--fps", type=float, default=15.0)
    movie_parser.add_argument("--codec", type=str, default="mp4v")
    movie_parser.add_argument("--use-annotated", action="store_true")
    movie_parser.add_argument("--overlay-tags", action="store_true")
    movie_parser.add_argument("--overlay-frame-index", action="store_true")

    args = parser.parse_args()

    if args.command in {None, "list"}:
        sys.exit(_print_sessions(list_movie_sessions(args.root_dir)))

    if args.command == "save-frames":
        result = save_movie_frames(
            camera=args.camera,
            duration_s=args.duration,
            max_frames=args.max_frames,
            root_dir=args.root_dir,
            session_name=args.session_name,
            pixels_per_cm=args.pixels_per_cm,
            diff_threshold=args.diff_threshold,
            min_interval_ms=args.min_interval_ms,
            max_gap_s=args.max_gap_s,
            image_format=args.image_format,
            jpeg_quality=args.jpeg_quality,
        )
        print(json.dumps(result, indent=2, sort_keys=True))
        sys.exit(0)

    if args.command == "make-movie":
        result = make_movie(
            session=args.session,
            root_dir=args.root_dir,
            output_path=args.output,
            fps=args.fps,
            codec=args.codec,
            use_annotated=args.use_annotated,
            overlay_tags=args.overlay_tags,
            overlay_frame_index=args.overlay_frame_index,
        )
        if isinstance(result, list):
            sys.exit(_print_sessions(result))
        print(json.dumps(result, indent=2, sort_keys=True))
        sys.exit(0)

    parser.error(f"Unknown command: {args.command}")


if __name__ == "__main__":
    main()
