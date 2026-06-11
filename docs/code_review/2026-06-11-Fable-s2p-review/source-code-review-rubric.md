# Source Code Review Rubric for `source/`

Date: 2026-06-08

This rubric defines how the firmware in `source/` will be reviewed. It is built
from embedded-systems safety guidance, robotics software architecture guidance,
and the local architecture already documented for this project. The next review
phase should use this rubric as the evaluation contract, not as a license to
invent unrelated style preferences.

## Scope

Review the firmware under `source/`, including:

- `main.cpp`
- `app/`
- `robot/`
- `control/`
- `hal/`
- `types/`

The review should focus on whether the code is simple, cohesive, deterministic,
and physically trustworthy on a small embedded robot. Findings should identify
real risks and unnecessary complexity, especially where recent development may
have added extra state, indirection, or duplicate concepts.

## Research Basis

These sources shape the criteria below:

- [JPL Power of Ten](https://spinroot.com/p10/) emphasizes simple control flow,
  bounded loops, no dynamic allocation after initialization, small functions,
  narrow variable scope, checked return values, limited preprocessor use, limited
  pointer complexity, warnings, and static analysis.
- [Barr Group Embedded C Coding Standard](https://barrgroup.com/embedded-systems/books/embedded-c-coding-standard)
  frames embedded coding standards around reducing firmware bugs while improving
  maintainability and portability, with practical rules for data types,
  functions, preprocessor use, and variables.
- [SEI CERT C Coding Standard](https://cmu-sei.github.io/secure-coding-standards/sei-cert-c-coding-standard/)
  organizes review concerns around declarations, expressions, integer and
  floating-point behavior, arrays, strings, memory management, I/O, error
  handling, APIs, and concurrency.
- [ROS REP 103](https://www.ros.org/reps/rep-0103.html) calls out unit and frame
  conventions as integration-critical: SI units, right-handed frames, `x`
  forward, `y` left, `z` up, and documented exceptions.
- [ROS REP 105](https://www.ros.org/reps/rep-0105.html) distinguishes continuous
  odometry frames from global map frames and emphasizes clear frame authority.
- [WPILib command-based robotics docs](https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html)
  emphasize subsystems that hide hardware internals, actions with clear
  lifecycles, periodic execution, resource ownership, and prevention of multiple
  writers fighting over the same actuator.
- Local project docs, especially `docs/architecture.md`, `docs/kinematics-model.md`,
  and `docs/protocol-v2.md`, define this firmware's intended layer boundaries,
  static ownership model, differential-drive conventions, wire protocol, and
  command/control split.

## Review Philosophy

The review should prefer code that a tired human can reason about while the robot
is on the bench. A good design does not merely have abstractions; it has the
right few abstractions, each with a clear job.

The strongest code will have these properties:

- The main path from command input to motor output is visible and easy to trace.
- Objects are highly cohesive: their state and behavior belong together.
- Layer boundaries reduce reasoning burden instead of hiding important behavior.
- Variables have clear purpose, tight scope, and unit-bearing names where useful.
- State machines are explicit, finite, and easy to audit.
- Hardware failures, stale sensors, timeouts, and saturation are handled directly.
- Math reflects the physical robot and uses documented units and frames.
- The firmware remains deterministic: bounded loops, bounded buffers, bounded
  blocking, and no heap allocation during normal operation.

## Scoring Scale

Use this scale for each category:

| Score | Meaning |
| --- | --- |
| 3 | Strong. Simple, cohesive, locally reasoned, and aligned with embedded robotics constraints. |
| 2 | Acceptable. Some roughness, but understandable and not risky enough to require immediate change. |
| 1 | Weak. Adds avoidable complexity, hides important behavior, or creates plausible maintenance risk. |
| 0 | Failing. Creates correctness, safety, determinism, or severe maintainability risk. |

Severity for findings:

- Critical: can plausibly cause unsafe motion, hardware damage, memory corruption,
  unbounded execution, or complete loss of command/control authority.
- High: likely behavioral bug, incorrect physical model, serious state-machine
  ambiguity, or architecture drift that blocks reliable future work.
- Medium: maintainability or testability issue that can cause defects over time.
- Low: local clarity, naming, duplication, or style issue with limited blast radius.

## Rubric Categories

### 1. Modularity and Layering

Questions to ask:

- Does each module have one clear reason to change?
- Do dependencies flow down the intended architecture rather than sideways or up?
- Is hardware access isolated in HAL or robot ownership code?
- Is command parsing separate from control policy, hardware I/O, and navigation math?
- Are public interfaces narrow, purposeful, and named in robot-domain terms?
- Are there abstractions that exist only to forward calls without reducing
  complexity?
- Are there globals or singleton-like paths that bypass ownership boundaries?

Strong code has a short chain of ownership and dependency injection where useful.
Weak code spreads one responsibility across many files, or makes one class aware
of too many layers.

### 2. Cohesion of Objects and Classes

Questions to ask:

- Does each class own the state it mutates?
- Do most methods use the class's core state, or is the class just a bag of
  unrelated helpers?
- Are constructor dependencies explicit and stable?
- Are hardware objects, configuration, state machines, and protocol parsing kept
  in the objects that naturally own them?
- Are there parallel variables representing the same concept in different places?
- Are there classes whose names are broad, such as `Manager`, `Handler`, or
  `Controller`, but whose responsibilities are not sharply bounded?

Strong code feels like each object has a center of gravity. Weak code requires
the reader to remember which remote object owns the real state.

### 3. Simple, Straightforward Execution Path

Questions to ask:

- Can a reviewer trace command input to actuator output without jumping through
  unrelated layers?
- Is the normal path more obvious than the special cases?
- Are start, tick, completion, cancellation, and stop paths explicit?
- Are state transitions represented directly rather than encoded as fragile
  combinations of booleans?
- Are there duplicate schedulers, duplicate command concepts, or hidden callbacks
  that make timing hard to reason about?
- Are temporary variables doing useful naming or calculation work, rather than
  merely renaming another variable?

Strong code makes the common path boring and visible. Weak code makes the reader
build a mental simulator before understanding one robot action.

### 4. Readability and Local Reasoning

Questions to ask:

- Are functions short enough to hold in working memory?
- Are local variables declared near use, scoped tightly, and named for meaning?
- Do names include units where confusion is likely, such as `_mm`, `_mms`,
  `_ms`, `_deg`, `_cdeg`, or `_rad`?
- Is control flow simple, with limited nesting and clear error exits?
- Are macros, templates, casts, and pointer indirection rare and justified?
- Do comments explain non-obvious protocol, hardware, timing, or physics
  constraints rather than narrating the code?
- Can individual functions be reviewed without needing broad global context?

Strong code spends complexity only where the robot or hardware requires it. Weak
code spends complexity on bookkeeping, indirection, or cleverness.

### 5. Embedded Determinism and Resource Use

Questions to ask:

- Is there no heap allocation during normal operation?
- Are all loops bounded, especially in tick paths, parsers, sensor polling, and
  radio/serial draining?
- Are buffers statically sized or stack-bounded with clear length checks?
- Are string operations NUL-safe and length-aware?
- Are I2C, serial, radio, and sensor calls checked for failure where the result
  matters?
- Are tick paths free of unbounded blocking, sleeps, retries, and long I/O bursts?
- Is time arithmetic wrap-safe for `uint32_t` millisecond clocks?
- Are interrupt, callback, or concurrent data boundaries explicit and protected
  with the right volatile/atomic/critical-section behavior where needed?

Strong code has predictable time and memory use. Weak code depends on luck,
short inputs, always-present devices, or a perfectly quiet bus.

### 6. Safety, Failsafe Behavior, and Hardware Interaction

Questions to ask:

- Does every actuator have a clear stopped or neutral state?
- Do watchdogs and command timeouts stop motion promptly?
- Do hardware-missing and sensor-invalid paths fail safe rather than continuing
  with stale or zero-looking data?
- Are PWM, velocity, distance, angle, and servo commands clamped at the boundary
  closest to the hardware?
- Are saturation rules explicit, especially when preserving differential-drive
  curvature matters more than preserving speed?
- Are initialization order, device detection, and bus recovery visible?
- Are bench-test or diagnostic modes unable to accidentally leak into production
  behavior without being obvious?

Strong code makes the safe behavior the default behavior. Weak code requires a
reader to infer that some upstream caller will always do the safe thing.

### 7. Robotics and Control Correctness

Questions to ask:

- Are units and coordinate frames consistent with local docs and ROS-style
  conventions where applicable?
- Are conversions centralized or obviously local, rather than repeated with
  slightly different constants?
- Is track width treated as the differential-drive geometry parameter, not mixed
  with unrelated dimensions?
- Is odometry continuous and distinct from any global or resettable pose concept?
- Are sensor validity, staleness, calibration, and reset semantics explicit?
- Are control loops given sane `dt` values and protected against zero, negative,
  huge, or underflowed intervals?
- Are acceleration, deceleration, and saturation handled in a way that respects
  the physical robot?
- Are end conditions for distance, timed, streaming, and go-to modes precise and
  observable?

Strong code lets physics be seen in the code. Weak code hides units, duplicates
kinematics, or lets control state drift through unrelated modules.

### 8. Protocol, Error Handling, and Observability

Questions to ask:

- Does the parser reject bad input deterministically and report useful errors?
- Are protocol responses consistent with `docs/protocol-v2.md`?
- Are correlation IDs, async completions, and safety events preserved across the
  right command paths?
- Are invalid arguments, bad keys, missing devices, and range errors explicit?
- Are return values from non-void functions checked when failure changes behavior?
- Is telemetry sufficient to understand mode, sensor state, faults, and motion
  completion without stepping through the debugger?
- Are debug or diagnostic messages bounded and safe for embedded buffers?

Strong code makes failures inspectable. Weak code silently drops errors or turns
hardware failure into plausible-looking data.

### 9. Testability and Change Safety

Questions to ask:

- Can pure math, parsing, state transitions, and clamping be tested without real
  hardware?
- Are HAL boundaries fakeable or at least narrow enough for host-side tests?
- Are edge cases covered for parser errors, time wrap, sensor invalidity,
  saturation, and mode transitions?
- Does configuration have one authoritative source, with defaults and overrides
  easy to inspect?
- Are docs updated when architecture, protocol, units, or safety behavior change?
- Would a future change be localized, or would it require touching unrelated
  layers?

Strong code invites small tests. Weak code requires hardware and full-system
execution to learn whether a simple branch works.

### 10. Complexity Risk Inventory

During the review, keep a running inventory of complexity smells:

- Multiple owners for one concept, such as mode, pose, time, config, or reply
  channel.
- Boolean flag combinations that should be an enum state machine.
- Variables that exist only to mirror or rename another variable.
- Layers that only pass values through without enforcing an invariant.
- Repeated unit conversions or repeated constants.
- Hidden callbacks that make ordering unclear.
- Objects with many public mutators and few invariants.
- Long functions that combine parsing, hardware I/O, control math, and response
  formatting.
- Test or diagnostic code interleaved with normal runtime behavior.
- Comments that explain a workaround instead of a clear design rule.

These are not automatically findings. They become findings when they make the
code harder to reason about, create duplicate truth, or raise the risk of robot
misbehavior.

## Weighting for the Next Review

The review should weight the user's stated concerns heavily:

| Area | Weight |
| --- | ---: |
| Modularity, cohesion, and simple execution path | 40% |
| Embedded determinism, resource use, and safety | 25% |
| Robotics/control correctness | 20% |
| Testability, observability, and documentation fit | 15% |

This means a design that is safe but unnecessarily tangled should still receive
meaningful findings. The goal is not only working firmware; the goal is firmware
that remains understandable enough to trust.

## Review Output Format

The next phase should produce findings first, ordered by severity. Each finding
should include:

- Severity.
- File and function or class location.
- The specific risk.
- Evidence from the code.
- A minimal improvement direction.
- Which rubric category it maps to.

After findings, include a short score table by category and a concise summary of
the overall architecture health. Avoid broad rewrites in the review itself unless
the user asks for implementation.
