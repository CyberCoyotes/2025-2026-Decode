# FTC Team 11940 - Driver Control Guide

**Reed City Middle School | 2025-2026 Season | Demo / Recruiting Configuration**

---

## About This Configuration

This guide reflects the **single-driver demo bot** configuration: all primary functions live on one gamepad so one person can drive and demonstrate every robot capability without an operator. A future iteration will split controls back across two gamepads when the team competes with both a driver and an operator.

---

## 🎮 GAMEPAD 1 - DRIVER CONTROLS

### Driving

| Control | Function |
|---|---|
| **Left Stick** | Strafe (side-to-side and forward/backward movement) |
| **Right Stick** | Rotate (turn left/right) |
| **Left Trigger** | Precision Mode (slow speed while held, for fine positioning) |

### Intake & Eject

| Control | Function |
|---|---|
| **Left Bumper** | Intake — wheels run in, slides extend, indexer runs forward (sensor auto-holds artifact when detected) |
| **X Button** | Eject — wheels run out, slides extend, indexer reverses |

> **Note:** Slides extend automatically when intake runs and retract when stopped. Wheels continue briefly after slides retract to complete the artifact transfer. The REV V3 color sensor stops the top indexer motor when an artifact is detected (< 3 cm); the bottom motor continues to hold it in position, ready to feed the shooter.

### Shooting

| Control | Function |
|---|---|
| **Right Bumper** | Shoot at the currently selected preset |
| **Right Trigger** | Shoot at the currently selected preset (same as RB — will become vision-assisted shoot in a future update) |

### Shooting Presets

| Control | Function | Description |
|---|---|---|
| **A Button** | SHORT_RANGE | Close shots |
| **B Button** | MEDIUM_RANGE | Mid-range shots |
| **Y Button** | LONG_RANGE | Far shots |

The preset arrangement on the diamond corresponds roughly to distance: A (bottom, closest) → B (right, mid) → Y (top, farthest).

### Drive Modes & Heading

| Control | Function |
|---|---|
| **BACK Button** | Switch to Field-Centric mode |
| **START Button** | *(Reserved for future use — operator handoff, vision-align, etc.)* |
| **GUIDE Button** | Reset Heading (only meaningful in Field-Centric mode) |

> START is intentionally unbound. When a feature needs a top-level button binding in the future (e.g. vision-align toggle, operator-handoff mode), that's where it goes.

### Configuration Adjustment

| Control | Function |
|---|---|
| **D-Pad Up** | Increase base drive speed |
| **D-Pad Down** | Decrease base drive speed |
| **D-Pad Right** | Increase steering sensitivity |
| **D-Pad Left** | Decrease steering sensitivity |

---

## 📊 TELEMETRY INFORMATION

### During Operation — Key Status Indicators

**Drive:**
- Current heading mode (Field-Centric or Robot-Centric)
- Effective speed multiplier
- Robot heading (degrees)
- Position (X, Y coordinates in inches)

**Intake:**
- Wheel status (INTAKING, EJECTING, IDLE, FINISHING)
- Slide position (EXTENDED / RETRACTED)

**Index:**
- Status (INTAKING, FEEDING, REVERSING, IDLE)
- Distance sensor reading (cm)
- Artifact detected (✓ YES when distance < 3 cm)

**Shooter:**
- Current preset (SHORT_RANGE, MEDIUM_RANGE, LONG_RANGE)
- Status (IDLE, SPINNING_UP, AT_SPEED, REVERSING)
- Target RPM vs current RPM
- Ready indicator (✓ YES when at target)
- Hood position

---

## 💡 OPERATION TIPS

1. **Pick a preset before shooting.** Tap A, B, or Y to select the distance preset. The shooter doesn't spin up until you press Right Bumper or Right Trigger — selecting a preset only changes which RPM and hood angle the shooter *would* use.

2. **Watch for "Ready" before firing.** When you press Right Bumper, the flywheel spins up first. The indexer waits to feed until the shooter reports it's at target speed (or until a 2-second timeout, whichever comes first). The "Ready ✓" telemetry indicator shows you when the wait is over.

3. **Use precision mode for tight spots.** Hold Left Trigger to slow the robot down for detail driving — lining up to a specific angle for demos, navigating between obstacles, etc. Release to return to normal speed.

4. **Reset heading when starting a demo.** In Field-Centric mode, "forward" is always the direction the robot was pointing when heading was last reset. Press GUIDE before each demo so the audience sees "push stick up, robot goes away from us" as the consistent behavior.

5. **Field-Centric vs. Robot-Centric.**
   - **Field-Centric (default):** Robot moves relative to the field. Pushing the left stick up always moves the robot away from where the driver is standing, regardless of which way the robot is facing. Easier for new drivers; better for demos.
   - **Robot-Centric:** Robot moves relative to its own orientation. Pushing the stick up moves the robot in whatever direction its front is pointing. Useful for very precise approach maneuvers but harder to drive instinctively.
   - Press BACK to switch to Field-Centric if you ever toggle away from it.

6. **Speed adjustments save battery.** Drop base speed (D-Pad Down) for long demo sessions where you're not racing. Higher speed drains the battery faster and makes precise positioning harder.

7. **Eject is for "just in case."** During a normal demo, you shouldn't need it — the intake/shoot cycle handles artifacts cleanly. X is there for clearing a stuck artifact or correcting a mis-intake. If you find yourself using it frequently, something else is wrong (check telemetry).

---

## ⚠️ IMPORTANT NOTES

### REV V3 Color Sensor (Smart Artifact Detection)

- **Distance threshold:** 3.0 cm
- When the intake runs (Left Bumper), the sensor monitors for an artifact. Once detected, the top indexer motor stops automatically while the bottom motor continues holding the artifact in feed position.
- When shooting (Right Bumper or Right Trigger), the indexer feeds regardless of the sensor — this is intentional, since the artifact needs to clear past the sensor to reach the flywheel.
- Monitor "Artifact Detected" telemetry for real-time feedback on whether an artifact is loaded.

### Shooter Sequence

- **Flywheel must reach target RPM before the indexer feeds.** The 2-second timeout exists as a safety: if PIDF tuning is off or the battery is weak, the shot still fires after 2 seconds even if the flywheel isn't quite at speed. Shots fired during spin-up will be lower-energy than expected.
- **The shoot button must be held.** Releasing Right Bumper / Right Trigger stops both the flywheel and the indexer.

### Drive Mode Defaults

- **Default mode:** Field-Centric
- **Turbo was intentionally omitted** from this build. Demos benefit from predictable, consistent speed. Base speed of 0.85 with precision-on-left-trigger covers the useful range.

---

## 🔧 DEFAULT SETTINGS

| Setting | Value |
|---|---|
| Base Speed | 0.85 (85%) |
| Precision Speed | 0.30 (30%) |
| Sensitivity | 1.2 |
| Deadzone | 0.1 |
| Color Sensor Distance Threshold | 3.0 cm |
| Shooter Spin-Up Timeout | 2.0 seconds |
| Shooter RPM Increment (auton tuning) | 100 RPM |
| Hood Position Increment (auton tuning) | 0.05 |
| Reverse Flywheel RPM (jam clear) | -1000 RPM |

### Shot Presets

| Preset | Target RPM | Hood Position |
|---|---|---|
| SHORT_RANGE | 2200 | 0.30 |
| MEDIUM_RANGE | 2500 | 0.60 |
| LONG_RANGE | 2800 | 0.60 |

---

## 📞 SUPPORT

**OpMode Name:** Team 11940 TeleOp

**Source File:** `PrimeTeleOp.java`

**Hardware:** REV V3 Color Sensor, GoBilda Pinpoint odometry, Limelight 3A vision camera (vision integration coming in future update)

**Configuration:** Single-driver demo bot — all functions on Gamepad 1, Gamepad 2 unused

**Last Updated:** *date of commit*

---

*Print this guide and keep it at the driver station for quick reference during demos!*