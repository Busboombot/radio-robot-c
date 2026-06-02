---
id: '007'
title: Firmware Architecture Foundation
status: done
branch: sprint/007-firmware-architecture-foundation
use-cases:
- SUC-001
- SUC-002
- SUC-003
- SUC-004
- SUC-005
- SUC-006
issues:
- firmware-architecture-refactor.md
---
<!-- CLASI: Before changing code or making plans, review the SE process in CLAUDE.md -->

# Sprint 007: Firmware Architecture Foundation

## Goals

Restructure the firmware so ownership, the main loop, and the command
surface are clean before any later feature work builds on them:

- Move `MicroBit` ownership out of `Robot` into `main.cpp` (file-scope
  singleton); construct `Robot` from references to the hardware.
- Make the main loop **visible in `main.cpp`**: per-iteration serial
  intake, radio intake, and a `robot.tick(now_ms, sink)` at the
  configured cadence. Track the **active reply sink** so robot-driven
  async completions (`T+DONE`/`D+DONE`/`G+DONE`/`SAFETY_STOP`) and
  telemetry route back on the channel the command arrived on (fixes the
  hardwired-serial bug).
- Give `Robot` a public **action/query interface** + **component
  accessors** (`config()`, `motor()`, `driveController()`, `odometry()`,
  `otos()`, `lineSensor()`, `colorSensor()`, `gripper()`, `portIO()`) +
  a `tick()` (no `while` loop in `Robot`).
- Reduce `CommandProcessor` to a **pure parse-and-dispatch** layer
  (`Robot& _robot` + `process(line, sink)`); strip its drive state,
  watchdog, streaming, odometry-delta, gripper state, hardware pointers,
  and `init(...)`.
- Unify `CommandProcessor::Params` + `CalibParams` into one **`RobotConfig`**
  (single `mmPerDegL/R`, single `trackwidthMm`) owned by `Robot` and
  passed by reference to subsystems — one source of truth, no divergence.
- Extract a **`DriveController`** (`source/control/DriveController.{h,cpp}`)
  holding the S/T/D/G state machines, S-watchdog, and streaming counter.
- Introduce the **multi-rate scheduler scaffolding** that the kinematics
  sprints fill in.

## Issues Addressed

- `firmware-architecture-refactor.md` — ownership, Robot interface, thin
  CommandProcessor, visible main loop, unified `RobotConfig`,
  `DriveController`, scheduler scaffolding.

## Rationale for Grouping

This is the **keystone sprint**. Every later sprint builds directly on
the new ownership structure, the unified `RobotConfig`, the
`DriveController`, and the scheduler scaffolding introduced here. It is a
single cohesive refactor issue and stands alone as its own sprint so the
foundation is reviewed and stabilized before feature work lands on it.

## Dependency Notes

- **Depends on:** none.
- **Blocks:** 008 (RobotConfig + Motor/VelocityController scaffolding),
  009 (clean parser for the v2 wire-format rewrite), 010 and 011
  (RobotConfig, DriveController, scheduler, single authoritative pose).
- Sequenced **before** the protocol v2 parser rewrite (009) per the
  issue's locked stakeholder decision.

## Tickets

| # | Title | Depends On |
|---|-------|------------|
| 001 | Unify RobotConfig — merge CalibParams and CommandProcessor::Params | — |
| 002 | Move MicroBit ownership to main.cpp — Robot takes peripheral refs | 001 |
| 003 | Extract DriveController — move S/T/D/G state machines out of CommandProcessor | 001, 002 |
| 004 | Visible main loop and reply-sink routing — fix async-completion channel bug | 002, 003 |
| 005 | Thin CommandProcessor — add Robot public interface and component accessors | 003, 004 |
| 006 | Cleanup — delete dead structs, null-cal paths, and deprecated fallbacks | 005 |

Tickets execute serially in the order listed.
