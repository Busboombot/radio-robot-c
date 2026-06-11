# Firmware Code Review Plan

Date: 2026-06-08

This plan defines five focused review views for `source/`. Each view should be
conducted independently, using `source-code-review-rubric.md` and
`code-review-architecture.md` as the review contract. The review itself should
produce findings only after these views are applied; this document is only the
plan.

## 1. Architecture, Modularity, and Cohesion

Assess whether the code still matches the intended layer boundaries:
`main.cpp` composition, communication/parsing, `Robot` facade, control, HAL, and
plain shared types. Look for responsibilities that have drifted into the wrong
module, duplicate owners for the same state, pass-through layers that do not
enforce an invariant, and classes whose methods no longer form one cohesive
concept.

Primary question: can each object be summarized by one responsibility, and does
it own the state it mutates?

## 2. Command-to-Motion Execution Paths

Trace the live runtime paths described in the architecture doc: boot,
communication drain, command dispatch, streaming drive, distance drive, timed
drive, go-to, stop, completion, and safety-stop. Evaluate whether the normal path
is straightforward and whether state transitions are explicit enough to audit.

Primary question: can a reader follow one command from input line to motor output
without reconstructing hidden scheduler, callback, or state-machine behavior?

## 3. Embedded Runtime, Timing, and Concurrency

Review deterministic embedded behavior: bounded loops, fixed buffers, stack use,
no heap allocation in normal operation, checked hardware results, wrap-safe time
math, scheduler cadence, split-phase sensor/control work, interrupt or callback
boundaries, and blocking I/O risk. Treat concurrency broadly: CODAL callbacks,
radio/serial buffering, scheduler task ordering, and shared state between tasks.

Primary question: is control behavior bounded and predictable even when input,
radio, serial, or I2C behavior is imperfect?

## 4. Robotics Model, Numerical Methods, and Hardware Safety

Review the physical correctness of the robot code: units, left/right sign
conventions, differential-drive kinematics, odometry continuity, velocity and
distance control, saturation, acceleration/deceleration, sensor validity,
calibration ownership, actuator clamping, watchdog behavior, and safe stop paths.
Review the numerical methods behind those behaviors as a first-class concern:
velocity estimation, velocity integration into distance or pose, acceleration
calculation, PID update math, time-step handling, accumulated error, saturation
interaction, and whether the chosen integration method is accurate enough for
high-quality motion. Prefer explicit, justified integration choices over casual
summation; call out places where Euler integration is the floor and a midpoint,
trapezoidal, exact-arc, or other method would materially improve accuracy.

Primary question: does the code make the robot's physical meaning, numerical
accuracy, and fail-safe behavior obvious at the point where decisions are made?

## 5. Interpretability, Dead Code, and Change Safety

Inspect for code that makes the firmware harder to reason about without adding
behavioral value: dead or obsolete paths, unused helpers, diagnostic code in
production paths, unnecessary temporaries, broad public mutators, duplicated
constants, unclear names, and abstractions that obscure rather than clarify.
Also note where missing tests or weak observability make future changes risky.

Primary question: what can be removed, renamed, localized, or tested so the next
reader can reason about the firmware with less mental bookkeeping?

## Review Output

The review should report findings first, ordered by severity. Each finding should
include the view that found it, concrete file/function evidence, the risk, and a
minimal correction direction. After findings, include a brief score table across
the five views and a short architecture-health summary.