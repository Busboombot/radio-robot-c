"""
firmware.py — ctypes loader and Sim class for the host simulation library.

Produced by ticket 020-004.  Loads libfirmware_host.dylib (macOS) or
libfirmware_host.so (Linux) and exposes all sim_* C ABI functions through
a Sim context manager.
"""
import ctypes
import pathlib
import sys

_HERE = pathlib.Path(__file__).parent


def _lib_name() -> str:
    return "libfirmware_host.dylib" if sys.platform == "darwin" else "libfirmware_host.so"


LIB_PATH = _HERE / "build" / _lib_name()


class Sim:
    """Context manager wrapping one SimHandle (MockHAL + Robot + CommandProcessor)."""

    def __init__(self) -> None:
        self._lib = ctypes.CDLL(str(LIB_PATH))
        self._setup_types()
        self._h = self._lib.sim_create()
        if not self._h:
            raise RuntimeError("sim_create() returned NULL")
        self._t: int = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def __enter__(self) -> "Sim":
        return self

    def __exit__(self, *_) -> None:
        if self._h:
            self._lib.sim_destroy(self._h)
            self._h = None

    # ------------------------------------------------------------------
    # Time advance
    # ------------------------------------------------------------------

    def tick_for(self, total_ms: int, step_ms: int = 24) -> None:
        """Advance simulation by total_ms milliseconds in step_ms increments."""
        end = self._t + total_ms
        while self._t < end:
            self._lib.sim_tick(self._h, ctypes.c_uint32(self._t))
            self._t += step_ms

    # ------------------------------------------------------------------
    # Command dispatch
    # ------------------------------------------------------------------

    def send_command(self, line: str) -> str:
        """Send one command line; return the synchronous reply as a decoded string."""
        buf = ctypes.create_string_buffer(512)
        n = self._lib.sim_command(self._h, line.encode(), buf, 512)
        if n <= 0:
            return ""
        return buf.raw[:n].decode(errors="replace")

    def get_async_evts(self) -> str:
        """Return any async EVT replies accumulated since the last send_command call."""
        buf = ctypes.create_string_buffer(2048)
        n = self._lib.sim_get_async_evts(self._h, buf, 2048)
        if n <= 0:
            return ""
        return buf.raw[:n].decode(errors="replace")

    # ------------------------------------------------------------------
    # Internal: argtypes / restype declarations
    # ------------------------------------------------------------------

    def _setup_types(self) -> None:
        lib = self._lib

        # sim_create() → void*
        lib.sim_create.argtypes = []
        lib.sim_create.restype = ctypes.c_void_p

        # sim_destroy(void* h)
        lib.sim_destroy.argtypes = [ctypes.c_void_p]
        lib.sim_destroy.restype = None

        # sim_tick(void* h, uint32_t now_ms)
        lib.sim_tick.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
        lib.sim_tick.restype = None

        # sim_command(void* h, const char* line, char* out_buf, int out_len) → int
        lib.sim_command.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_int,
        ]
        lib.sim_command.restype = ctypes.c_int

        # sim_get_enc_l / sim_get_enc_r → float
        lib.sim_get_enc_l.argtypes = [ctypes.c_void_p]
        lib.sim_get_enc_l.restype = ctypes.c_float
        lib.sim_get_enc_r.argtypes = [ctypes.c_void_p]
        lib.sim_get_enc_r.restype = ctypes.c_float

        # sim_get_vel_l / sim_get_vel_r → float
        lib.sim_get_vel_l.argtypes = [ctypes.c_void_p]
        lib.sim_get_vel_l.restype = ctypes.c_float
        lib.sim_get_vel_r.argtypes = [ctypes.c_void_p]
        lib.sim_get_vel_r.restype = ctypes.c_float

        # sim_get_pwm_l / sim_get_pwm_r → float
        lib.sim_get_pwm_l.argtypes = [ctypes.c_void_p]
        lib.sim_get_pwm_l.restype = ctypes.c_float
        lib.sim_get_pwm_r.argtypes = [ctypes.c_void_p]
        lib.sim_get_pwm_r.restype = ctypes.c_float

        # sim_get_pose_x / sim_get_pose_y / sim_get_pose_h → float
        lib.sim_get_pose_x.argtypes = [ctypes.c_void_p]
        lib.sim_get_pose_x.restype = ctypes.c_float
        lib.sim_get_pose_y.argtypes = [ctypes.c_void_p]
        lib.sim_get_pose_y.restype = ctypes.c_float
        lib.sim_get_pose_h.argtypes = [ctypes.c_void_p]
        lib.sim_get_pose_h.restype = ctypes.c_float

        # sim_set_enc_l(void* h, float mm)
        lib.sim_set_enc_l.argtypes = [ctypes.c_void_p, ctypes.c_float]
        lib.sim_set_enc_l.restype = None

        # sim_set_enc_r(void* h, float mm)
        lib.sim_set_enc_r.argtypes = [ctypes.c_void_p, ctypes.c_float]
        lib.sim_set_enc_r.restype = None

        # sim_set_otos_pose(void* h, float x, float y, float hrad)
        lib.sim_set_otos_pose.argtypes = [
            ctypes.c_void_p,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
        ]
        lib.sim_set_otos_pose.restype = None

        # sim_set_motor_offset(void* h, int side, float factor)
        lib.sim_set_motor_offset.argtypes = [
            ctypes.c_void_p,
            ctypes.c_int,
            ctypes.c_float,
        ]
        lib.sim_set_motor_offset.restype = None

        # sim_get_async_evts(void* h, char* evts_buf, int evts_len) → int
        lib.sim_get_async_evts.argtypes = [
            ctypes.c_void_p,
            ctypes.c_char_p,
            ctypes.c_int,
        ]
        lib.sim_get_async_evts.restype = ctypes.c_int
