# Architecture: Command-Based Rewrite

> **Read this file at the start of every Claude Code session.**
> If a session ends with code that contradicts this doc, the doc wins.
> If a decision isn't in this doc, it doesn't exist yet — ask before inventing one.

> **Location:** This file lives at the **repository root** — same level as `FtcRobotController/`, `TeamCode/`, `build.dependencies.gradle`, `.gitattributes`, and `settings.gradle`. Not inside `TeamCode/`.

---

## 1. Strategic context

We are rewriting the 2025-2026 DECODE codebase from iterative (`LinearOpMode` + edge-detect booleans + hand-rolled state) into command-based (FTCLib `CommandOpMode` + triggers + command classes + status-enum subsystems). The rewrite serves three goals:

1. **2027 platform alignment.** FTC moves to WPILib on REV SystemCore/MotionCore. Command-based is the WPILib paradigm. Investing now means less to relearn next year.
2. **New programmers in September.** All current FTC programmers age out or graduate. New middle schoolers arrive with no legacy mental model. Whatever pattern we lock in now is the pattern they'll learn as "how robot code works." Make it the pattern that scales.
3. **Eliminate accumulated smells.** Cross-subsystem mutable state (`flywheelRunning`), subsystems accepting other subsystems' state as parameters (`setFlywheelOverride`), 7+ `lastXxxState` booleans per OpMode, hand-rolled scheduling with `sleep()` and millisecond timestamps. All gone in the rewrite.

**Audience for the resulting code:**

- **TeleOp** is read and modified by middle schoolers (grades 6-8). Must be legible.
- **Auton** is owned by the mentor with occasional student involvement. Can be denser.
- **Subsystems** are reference material everyone reads but few rewrite mid-season.
- **Commands** are written by the mentor early-season, then students add new ones following the templates.

**The 2-team / 1-codebase setup stays.** Product flavors `team11940` and `team22091` build separate APKs from one shared `src/main`. The teams are physically identical; flavor differences are per-robot tuning values and the `@TeleOp` / `@Autonomous` display names only.

---

## 2. MUST — non-negotiable conventions

### Subsystem pattern

Every subsystem MUST:

- Live in `common/subsystems/` (shared) — never in a flavor source set.
- Expose a public `Status` enum with gerund names (`INTAKING`, `SPINNING_UP`, `AT_SPEED`).
- Provide `public Status getStatus()`.
- Provide boolean predicates for common queries (`isIdle()`, `isReady()`, `hasArtifact()`).
- Provide command methods named as verbs (`intake()`, `feed()`, `stop()`, `shoot()`).
- Provide a `periodic()` method called by the scheduler. Even if empty, it's there.
- Store hardware config name strings as `private static final String` at the top of the class.
- Derive status from existing state where possible (no redundant state variables to keep in sync).

### Command pattern

Every command MUST:

- Live in `common/commands/` (shared, default) or `team11940/commands/` / `team22091/commands/` (per-team).
- Extend `CommandBase` from FTCLib.
- Declare required subsystems in the constructor via `addRequirements(...)`.
- Override `initialize()`, `execute()`, `isFinished()`, `end(boolean interrupted)` as needed.
- Be a verbose class file when stateful or longer than ~5 lines of logic.
- Be a factory call (`Commands.runOnce(...)`, `Commands.run(...)`) only when truly trivial.

### Naming

- **Classes:** Nouns. `ShooterSubsystem`, `ShootCommand`, `Team11940Config`.
- **Status enum values:** Gerunds. `INTAKING`, `SPINNING_UP`, `FEEDING`.
- **Command methods on subsystems:** Action verbs. `intake()`, `shoot()`, `stop()`, `reset()`.
- **Setters:** `set` + property. `setHoodPosition()`, `setPreset()`.
- **Getters:** `get` + property, OR `is`/`has` + property for booleans. `getCurrentRPM()`, `isReady()`, `hasArtifact()`.
- **Commands:** Verb + `Command`. `ShootCommand`, `IntakeCommand`, `ClearJamCommand`.

### Lambdas

- Always use the arrow form: `() -> shooter.isReady()`.
- Never use anonymous inner classes for functional interfaces.
- Never use method references when a multi-line lambda would be clearer.

### Triggers

- Replace every `lastXxxState` edge-detect boolean with an FTCLib `Trigger`.
- Use `new Trigger(() -> condition)` for non-button conditions.
- Use `gamepadEx.getGamepadButton(GamepadKeys.Button.X)` for buttons.
- Combo buttons: `triggerA.and(triggerB).onTrue(command)`.
- Bind in `bindCommands()` (or equivalent setup method), not scattered through the OpMode.

### File structure

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── common/
│   ├── config/          RobotConfig interface + Pedro/auton constants
│   ├── subsystems/      All shared subsystems (Shooter, Index, Intake, Drive, etc.)
│   ├── commands/        All shared commands (ShootCommand, IntakeCommand, etc.)
│   ├── teleop/          PrimeTeleOpBase (abstract CommandOpMode)
│   ├── auton/           PedroAutonBase + shared auton commands/paths
│   └── pedropathing/    PedroConfig, Tuning, TUNING.md, paths/
├── team11940/           Team11940Config, TeleOp wrapper (@TeleOp name + getRobotConfig())
└── team22091/           Team22091Config, TeleOp wrapper (@TeleOp name + getRobotConfig())
```

### Dependencies

- Already in `build.dependencies.gradle`: Pedro Pathing 2.1.2, FTCLib 2.1.1, Pedro telemetry, Bylazar Panels.
- **No new dependencies without updating this doc first.**

### Read-only paths — do not modify

The following are off-limits to Claude Code rewrite sessions. They may be read for reference (especially `FtcRobotController/samples/` for SDK usage examples), but never edited.

| Path | Why off-limits |
|---|---|
| `FtcRobotController/` (entire directory) | FTC SDK. Upgraded as a unit when a new SDK release ships. Local edits get clobbered on the next upgrade and break the build in subtle ways. |
| `gradle/`, `gradlew`, `gradlew.bat` | Gradle wrapper. Comes from the SDK template. |
| `.idea/`, `.vscode/` | IDE workspace settings (already gitignored). |
| `local.properties` | Per-machine SDK paths. Never committed. |
| Root `build.gradle`, `build.common.gradle`, `settings.gradle` | Root Gradle config. Touch only when explicitly approved (e.g. adding a flavor or product). |

`build.dependencies.gradle` at the root and `TeamCode/build.gradle` **can** be modified, but only when the task explicitly involves adding a dependency or changing flavor configuration. Default assumption: leave them alone.

The one historical exception was the `.gitattributes` + `git add --renormalize .` line-ending fix, which intentionally touched FtcRobotController files to normalize CRLF → LF. That was a deliberate one-time repo-hygiene operation. It does not generalize.

### Hardware config names

Hardware configuration string names are **immutable** without explicit approval. See section 5. Changing a name silently breaks the driver-station configuration on the physical robot.

---

## 3. MUST NOT — anti-patterns

These are the smells we're rewriting away from. Do not reintroduce them.

- **No shared mutable state between subsystems.** TeleOp must not track flags like `flywheelRunning` and push them into other subsystems. If subsystem A needs to know about subsystem B's state, A queries B's `getStatus()` — but ideally A doesn't need to know at all, and the command layer makes the cross-cutting decision.
- **No subsystem accepting another subsystem's state as a parameter.** `setFlywheelOverride(boolean)` is the canonical example. Delete on sight.
- **No `lastXxxState` edge-detect booleans.** Triggers handle this.
- **No `sleep()` calls in commands or subsystems.** Use `WaitCommand` or check elapsed time via the command's own timer.
- **No factory chains longer than 3 lines.** If it grows, extract to a command class.
- **No exception handling unless explicitly requested.** Don't wrap motor calls in try/catch defensively.
- **No reformatting unrelated files.** Modify only what the current task names.
- **No subsystem-to-subsystem hard dependencies in constructors.** Subsystems are constructed independently from `HardwareMap`. Commands compose them.
- **No `@Disabled` annotations on production OpModes** (used freely on test OpModes).
- **No silent name changes.** Hardware names, file names, class names, method names locked once committed.
- **No reinventing conventions.** If something isn't in this doc, ask. Don't pick.

---

## 4. Reference implementation — the gold standard

**`ShooterSubsystem` is the gold-standard subsystem.**

Every other subsystem must match its pattern:
- Public `Status` enum
- `getStatus()` derived from existing state where possible
- Predicates (`isReady()`, `isIdle()`)
- Verb command methods (`shoot()`, `stop()`)
- Hardware names as `private static final String` constants
- No knowledge of other subsystems

**`ShootCommand` is the gold-standard command.** Write `IntakeCommand` first to validate the pattern (one subsystem, simple lifecycle), then write `ShootCommand` as the template all other commands follow. `ShootCommand` cannot be written until `ShooterSubsystem` and `IndexSubsystem` are rewritten with their `Status` enums.

Every other command must match its pattern:
- Extends `CommandBase`
- Constructor takes subsystems, calls `addRequirements()`
- `initialize()` for one-time setup
- `execute()` for per-tick work
- `isFinished()` for termination condition
- `end(boolean interrupted)` for cleanup

When in doubt during a rewrite session, re-read these two files. They are the source of truth for "what does our code look like."

---

## 5. Hardware inventory

### Drivetrain (Mecanum)

| Hardware name | Type | Notes |
|---|---|---|
| `leftFront` | DcMotorEx | goBilda |
| `leftBack` | DcMotorEx | goBilda |
| `rightFront` | DcMotorEx | goBilda, reversed |
| `rightBack` | DcMotorEx | goBilda, reversed |

### Odometry

| Hardware name | Type | Notes |
|---|---|---|
| `odo` | GoBildaPinpointDriver | I2C; do NOT instantiate twice in one OpMode |

- Pods: goBilda swingarm
- Forward pod: 95.25mm right of center (Pedro `forwardPodY = -3.75"`)
- Strafe pod: centered front-to-back (`strafePodX = 0.0"`)
- Forward encoder direction: `FORWARD` (Pinpoint mounted upside-down)

### Shooting system

| Hardware name | Type | Notes |
|---|---|---|
| `flywheelMotor` | DcMotorEx | goBilda 5203 6000 RPM 1:1, reversed |
| `hoodServo` | Servo | Hood angle adjustment |
| `indexBottomMotor` | DcMotor | Bottom indexer stage, reversed |
| `indexTopMotor` | DcMotor | Top indexer stage, reversed |
| `indexColorSensor` | DistanceSensor | REV V3 Color Sensor for artifact detection |

### Intake

| Hardware name | Type | Notes |
|---|---|---|
| `intakeWheelLeft` | Servo (continuous) | |
| `intakeWheelRight` | Servo (continuous) | Reversed |
| `intakeSlideLeft` | Servo (position) | Reversed |
| `intakeSlideRight` | Servo (position) | Reversed |

### Vision

| Hardware name | Type | Notes |
|---|---|---|
| `limelight` | Limelight3A | IP `192.168.1.11`, pipeline 0 = AprilTag 36h11 |

### Behavior thresholds (locked unless explicitly retuned)

| Constant | Value | Used by |
|---|---|---|
| Color sensor distance threshold | 3.0 cm | IndexSubsystem |
| Intake wheel stop delay after slide retract | 300 ms | IntakeSubsystem |
| Index bottom motor delay after intake combo | 900 ms | TeleOp |
| Shooter RPM manual increment | 100 RPM | TeleOp |
| Hood position manual increment | 0.05 | TeleOp |
| Flywheel CPR (Yellow Jacket 5203) | 28.0 | ShooterSubsystem |
| Default drive speed | 0.85 | RobotConfig defaults |
| Default sensitivity | 1.2 | RobotConfig defaults |
| Default deadzone | 0.1 | RobotConfig defaults |
| Reverse flywheel RPM (jam clear) | -1000 | ShooterSubsystem |

### Shot presets (locked unless retuned)

| Preset | Target RPM | Hood position |
|---|---|---|
| `SHORT_RANGE` | 2200 | 0.30 |
| `MEDIUM_RANGE` | 2500 | 0.60 |
| `LONG_RANGE` | 2800 | 0.60 |

> **Known code drift:** `ShooterSubsystem.ShotPreset.LONG_RANGE` is currently hardcoded to 3500 RPM (experimental value). Fix to 2800 when rewriting ShooterSubsystem.

---

## 6. Behavior specification

Written in English. No code. This is what the robot does, not how. If a rewrite produces code that violates a behavior here, the code is wrong.

### Drivetrain

- Mecanum drive with field-centric and robot-centric modes, default field-centric.
- Field-centric uses Pinpoint heading (preferred) with fallback to IMU.
- Driver gamepad controls: left stick translation, right stick rotation.
- Driver can toggle field-centric/robot-centric, toggle turbo mode, reset heading, increase/decrease speed and sensitivity.
- Emergency stop: holding a button freezes all drive motors.

### Intake

- Wheels run forward to collect, reverse to eject, idle when not commanded.
- Slides auto-extend when wheels start running (any direction).
- Slides retract when wheels stop being commanded.
- Wheels continue running for 300 ms after slides begin retracting (artifact transfer completion).
- Slides have manual override mode that disables auto-extend/retract.

### Index

- Two motors (bottom + top) that run together at full speed in commanded direction.
- **`INTAKING` state:** Forward; top motor stops when distance sensor reads < 3 cm (artifact captured and held by bottom motor).
- **`FEEDING` state:** Forward; distance sensor is ignored (feeding artifact into shooter).
- **`REVERSING` state:** Both motors backward for jam clearing.
- **`IDLE` state:** Both motors off.

### Shooter

- One flywheel motor with velocity-PIDF control, one hood servo.
- Three shot presets (`SHORT_RANGE`, `MEDIUM_RANGE`, `LONG_RANGE`) each setting both flywheel RPM and hood position.
- Manual fine-tune: ±100 RPM, ±0.05 hood per press.
- Status derived: `IDLE` (target = 0), `SPINNING_UP` (target > 0, not at RPM), `AT_SPEED` (within 5% of target), `REVERSING` (target < 0).
- Reverse mode runs flywheel at -1000 RPM for jam clearing.

### Shoot sequence (composite behavior)

1. Operator presses RB.
2. Shooter commanded to current preset (spins up).
3. Wait for `AT_SPEED` OR 2-second timeout — whichever comes first. Index fires regardless after the wait.
4. Index commanded to `FEEDING`; continues until RB is released.
5. On release of RB, shooter returns to `IDLE`, index returns to `IDLE`.

### Jam clear (composite behavior)

- RB + A together: shooter and index both REVERSING.
- LB + A together: index only REVERSING.
- Releasing the combo returns both to IDLE.

### Vision (Limelight)

- Pipeline 0 = AprilTag 36h11.
- Provides botpose for localization fusion (see section 7).
- Provides tx/ty for vision-based alignment commands.
- Always available for telemetry display; consumed only when explicitly commanded.

### Autonomous

- Pedro Pathing 2.1.2 follows pre-designed paths.
- Paths designed in the visualizer at `https://visualizer.pedropathing.com/` and exported.
- `.pp` files committed to `paths/` directory alongside the Java that uses them.
- Auton OpModes extend `PedroAutonBase` and define start pose, scoring path, parking path.
- State machine in auton tracks path progress; scoring commands triggered at path waypoints.

---

## 7. Pose fusion (Limelight + Pinpoint)

**Deferred — not part of the command-based rewrite.**

All OpModes use raw Pinpoint pose during the rewrite. Limelight remains available for telemetry and vision-alignment commands but does not feed the pose estimate.

Pose fusion is a follow-on project after the rewrite is stable. When that project starts, open a new issue and design it there rather than in this document.

---

## 8. Rewrite session protocol

Every Claude Code session for this rewrite starts with:

> Read `ARCHITECTURE.md` at the repo root. Read the gold-standard files (`ShooterSubsystem.java` + `ShootCommand.java`). Then [TASK]. Modify only the files listed in the task. Do not modify anything under `FtcRobotController/`, the Gradle wrapper, or root-level Gradle files unless the task explicitly says so. Ask before adding dependencies, changing hardware names, or inventing conventions. **If you encounter a `[CONFIRM]` marker in `ARCHITECTURE.md` that is relevant to the current task, stop and ask before proceeding — these are decisions I have not yet made.**

One subsystem or one command per session. Do not batch. A 30-minute focused session produces better code than a 3-hour kitchen-sink session.

After each session: commit, push, update the GitHub issue with the commit hash, close the issue, and look at the diff before starting the next one. If something drifted from the doc, fix the doc OR fix the code — but make them agree before moving on.

---

## 9. Open decisions ([CONFIRM] checklist)

`[CONFIRM]` markers throughout this doc flag decisions that are not yet locked. Claude Code is instructed (section 8) to stop and ask when it hits one relevant to the current task. When a decision is made, replace the `[CONFIRM]` with the chosen answer and note the change in the commit message.

- [x] `ShooterSubsystem` is the gold standard
- [x] `IntakeCommand` is written first (pattern validation); `ShootCommand` is written second and becomes the gold-standard template
- [x] Shoot sequence: wait for `AT_SPEED` OR 2-second timeout (whichever first), then feed regardless; command ends on button release
- [x] Shoot sequence timeout value: 2 seconds (spin-up wait only, not total command duration)
- [x] Pose fusion: deferred; raw Pinpoint pose used throughout the rewrite; fusion is a separate follow-on project
- [x] `IntakeSubsystem` stays as one class (wheels + slides); auto-slide coupling is intentional physical behavior, not a command-layer concern
- [x] Existing button bindings carry forward unchanged into the rewrite
- [x] All commands live in `common/commands/`; no team-specific command directories. Team identity is fully expressed by the `@TeleOp` display name (in the flavor wrapper) and tuning values in `RobotConfig`.

---

## 10. Anti-drift checklist for the mentor

At the end of each session, before merging:

- [ ] New code matches the gold-standard pattern (status enum, predicates, verb methods)
- [ ] No `lastXxxState` booleans introduced
- [ ] No shared mutable state between subsystems
- [ ] No `sleep()` in commands
- [ ] Lambdas use arrow form consistently
- [ ] Files in correct directories per section 2
- [ ] **No files modified under `FtcRobotController/` or the Gradle wrapper**
- [ ] Hardware names unchanged from section 5
- [ ] If the code does something this doc forbids, either the doc or the code is wrong — resolve before merging

---

*Last updated: [date of commit]. When this doc changes, note what changed and why in the commit message.*