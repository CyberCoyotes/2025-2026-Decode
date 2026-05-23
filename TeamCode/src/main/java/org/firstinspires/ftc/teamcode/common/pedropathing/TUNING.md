# Pedro Pathing 2.1.2 — Drivetrain Tuning Guide

**Robot:** Teams 11940 & 22091 (identical hardware)  
**Before you start:** Build and deploy the APK to the Control Hub with Android Studio. The `Tuning` OpMode must be on the robot.

---

## What you need

- Robot on the floor with room to move (~6 feet clear in all directions)
- Driver Hub connected to the Control Hub via Wi-Fi Direct
- Laptop with Android Studio open to this repo (to update constants after each step)
- Tape measure
- Charged battery

---

## Overview

Tuning happens in two phases. **Phase 1 is just pushing the robot by hand — no motors run.** Phase 2 is the robot driving itself.

| Phase | Step | Robot moves? |
|---|---|---|
| 1 | Verify forward localization | No — you push it |
| 1 | Verify strafe localization | No — you push it |
| 2 | Forward velocity tuner | Yes |
| 2 | Strafe velocity tuner | Yes |
| 2 | Forward zero turn | Yes |
| 2 | Strafe zero turn | Yes |
| 2 | Turn test | Yes (spins in place) |

---

## How to run the Tuning OpMode

1. On the Driver Hub, tap the three-bar menu → **TeleOp** list
2. Select **"Tuning"** (group: Pedro Pathing)
3. Init the OpMode — you will see a menu on the Driver Hub screen
4. Use the **D-Pad up/down** to select a sub-mode
5. Press **Start** to run it
6. Read the telemetry values on screen — the numbers you care about are noted in each step below

---

## Phase 1 — Localization Verification

> These steps confirm the Pinpoint is reading the right directions before any motors run.  
> The robot will not move on its own.

### Step 1 — Forward Localizer

1. Select **"ForwardLocalizer"** from the Tuning menu
2. Place the robot pointing forward (0° heading)
3. **Push the robot straight forward** about 3–4 feet
4. Watch the telemetry: **X value should increase**

**If X decreases:** Open `PedroConfig.java` and flip `forwardEncoderDirection`:
```java
// Change this:
.forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
// To this:
.forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
```
Redeploy and re-run before continuing.

---

### Step 2 — Strafe Localizer

1. Select **"StrafeLocalizer"** from the Tuning menu
2. **Push the robot left** about 3–4 feet
3. Watch the telemetry: **Y value should increase**

**If Y decreases:** Open `PedroConfig.java` and flip `strafeEncoderDirection`:
```java
// Change this:
.strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
// To this:
.strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
```
Redeploy and re-run before continuing.

---

## Phase 2 — Drivetrain Tuning

> The robot will drive itself. Keep hands clear. Have someone ready to stop the OpMode if the robot behaves unexpectedly.

### Step 3 — Forward Velocity Tuner

1. Clear ~8 feet of space directly in front of the robot
2. Select **"ForwardVelocityTuner"** from the Tuning menu
3. The robot will drive forward and back several times
4. When it stops, read **`xVelocity`** from telemetry

**Update `PedroConfig.java`:**
```java
public static final MecanumConstants MECANUM_CONSTANTS = new MecanumConstants()
    // ... motor names/directions stay the same ...
    .xVelocity(XX.XXXXX)   // ← replace with the reported value
```

---

### Step 4 — Strafe Velocity Tuner

1. Clear ~8 feet of space to the left and right of the robot
2. Select **"StrafeVelocityTuner"** from the Tuning menu
3. The robot will strafe left and right several times
4. When it stops, read **`yVelocity`** from telemetry

**Update `PedroConfig.java`:**
```java
    .yVelocity(XX.XXXXX)   // ← replace with the reported value
```

> After Steps 3 and 4: redeploy the APK before continuing to PID tuning. The velocity values affect PID tuning.

---

### Step 5 — Forward Zero Turn

1. Clear space in front (robot will drive forward repeatedly)
2. Select **"ForwardZeroTurn"** from the Tuning menu
3. The robot will run a forward-back translational test
4. Read the **translational PID values** from telemetry

**Update `FOLLOWER_CONSTANTS` in `PedroConfig.java`:**
```java
public static final FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants()
    .translationalPIDFCoefficients(new PIDFCoefficients(kP, 0, kD, 0))
    .translationalSmallPIDFCoefficients(new PIDFCoefficients(kP, 0, kD, 0));
```

---

### Step 6 — Strafe Zero Turn

1. Select **"StrafeZeroTurn"** from the Tuning menu
2. The robot will run a strafe translational test
3. Read the **lateral PID values** from telemetry

Update `FOLLOWER_CONSTANTS` with lateral PID values the same way as Step 5.

---

### Step 7 — Turn Test

1. Select **"TurnTest"** from the Tuning menu
2. The robot will spin in place
3. Read the **heading PID values** from telemetry

**Update `FOLLOWER_CONSTANTS` in `PedroConfig.java`:**
```java
    .headingPIDFCoefficients(new PIDFCoefficients(kP, 0, kD, 0))
    .headingSmallPIDFCoefficients(new PIDFCoefficients(kP, 0, kD, 0));
```

---

## After All Steps — Final Verification

1. Deploy the APK with all updated constants
2. Select **"StraightBackAndForth"** from the Tuning menu
3. The robot should drive a clean straight line and return to (nearly) the same spot
4. Select **"CurvedBackAndForth"** — robot should follow a smooth curved path

If the robot drifts noticeably, re-run Steps 5–7 and tighten the PID values (increase kP slightly, add a small kD if oscillating).

---

## Quick Reference — What to update after tuning

All changes go in one file: `common/pedropathing/PedroConfig.java`

```
PINPOINT_CONSTANTS  — only if encoder directions were wrong in Phase 1
MECANUM_CONSTANTS   — xVelocity (Step 3), yVelocity (Step 4)
FOLLOWER_CONSTANTS  — translational PID (Step 5), lateral PID (Step 6), heading PID (Step 7)
PATH_CONSTRAINTS    — after tuning, you can raise maxVelocity from 0.6 toward 0.9
```

---

## Common Problems

| Symptom | Likely cause | Fix |
|---|---|---|
| Robot drives sideways when it should go straight | Motor direction wrong | Check `leftBack`/`rightBack` directions in `MECANUM_CONSTANTS` |
| X or Y reads backwards in Phase 1 | Encoder direction | Flip the direction constant, redeploy |
| Robot oscillates / overshoots on paths | kP too high | Reduce kP in the relevant PID; add small kD |
| Robot undershoots / moves sluggishly | kP too low | Increase kP; check battery charge |
| Pinpoint shows `FAULT_NO_PODS_DETECTED` | Wiring or I2C name wrong | Verify I2C device is named `"odo"` in robot config |
| `Tuning` OpMode not in list | APK not deployed | Re-run Android Studio deploy |

---

## Hardware Reference (this robot)

| Item | Value |
|---|---|
| Pinpoint I2C name | `odo` |
| Pod type | GoBilda Swingarm |
| Pinpoint position | 3.75" right of center, centered front-to-back |
| Motor names | `leftFront`, `leftBack`, `rightFront`, `rightBack` |
| Left motor direction | REVERSE |
| Right motor direction | FORWARD |
