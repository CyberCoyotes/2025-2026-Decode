# Pedro Pathing Setup Guide - Team 11940

This guide will help you get started with the Pedro Pathing autonomous programs.

## What's Been Added

1. **Dependencies** (`build.dependencies.gradle`)
   - Pedro Pathing library v1.0.2
   - JitPack repository for Pedro Pathing

2. **Sample Autonomous Programs**
   - `PedroBlueSampleAuto.java` - Blue alliance autonomous
   - `PedroRedSampleAuto.java` - Red alliance autonomous

3. **Configuration**
   - `PedroPathingConstants.java` - Tuning parameters for your robot

## Quick Start

### 1. Sync Gradle
After pulling these changes, sync your Gradle files:
- In Android Studio: Click "Sync Now" or File → Sync Project with Gradle Files

### 2. Configure Pedro Pathing

The Pedro Pathing library will automatically detect and use your **Pinpoint Odometry Computer** if it's configured in your hardware map with the name `"odo"` or `"pinpoint"`.

Make sure your hardware configuration includes:
```
Control Hub Portal → Configure → [Your Robot Config] → Add: goBILDA Pinpoint
Device Name: "odo" or "pinpoint"
```

### 3. Tune Your Robot Parameters

Open `PedroPathingConstants.java` and tune these values for your robot:

#### Critical Parameters to Measure:

1. **Max Velocities** - Run velocity tuning OpMode
   ```java
   maxForwardVelocity = 50.0;  // Measure by driving full speed
   maxLateralVelocity = 40.0;   // Measure by strafing full speed
   ```

2. **Motor Directions** - Verify motors spin correctly
   ```java
   leftFrontDirection = DcMotorSimple.Direction.REVERSE;
   // etc.
   ```

3. **PID Values** - Start with defaults, then tune
   ```java
   translationalPIDkP = 0.2;  // Position following
   headingPIDkP = 2.0;        // Rotation control
   ```

### 4. Customize Autonomous Paths

Open `PedroBlueSampleAuto.java` or `PedroRedSampleAuto.java` and customize:

1. **Starting Position** - Where your robot starts
   ```java
   private final Pose START_POSE = new Pose(-12, -63, Math.toRadians(90));
   ```

2. **Waypoints** - Key positions on the field
   ```java
   private final Pose SAMPLE_1_POSE = new Pose(-48, -40, Math.toRadians(90));
   ```

3. **Autonomous Sequence** - What actions to perform
   - Modify `scorePreloadSpecimen()`, `collectAndScoreSamples()`, etc.

## Sample Autonomous Overview

Both autonomous programs follow this sequence:

1. **Score Pre-loaded Specimen**
   - Drive to submersible
   - Score specimen on high chamber

2. **Collect and Score Sample**
   - Drive to sample collection area
   - Activate intake to collect sample
   - Drive to high basket
   - Score sample using shooter

3. **Park**
   - Drive to observation zone
   - Complete autonomous

## Coordinate System

Pedro Pathing uses the field coordinate system:

```
Blue Alliance (Negative Y)
┌─────────────────────────────┐
│                             │
│  (-X, -Y)      (0,0)  (+X,-Y)│
│                             │
│                             │
│                             │
│  (-X, +Y)           (+X,+Y) │
│                             │
└─────────────────────────────┘
Red Alliance (Positive Y)
```

- **Origin (0, 0)**: Field center
- **X-axis**: Positive = Right (from your alliance wall)
- **Y-axis**: Positive = Forward (toward opponent)
- **Heading**: 0° = Facing forward, 90° = Facing left

## Path Types

Pedro Pathing supports different path types:

### BezierLine
Straight line between two points:
```java
Path straightPath = new Path(new BezierLine(
    new Point(startX, startY, Point.CARTESIAN),
    new Point(endX, endY, Point.CARTESIAN)
));
```

### BezierCurve
Smooth curve with control points:
```java
Path curvedPath = new Path(new BezierCurve(
    new Point(startX, startY, Point.CARTESIAN),
    new Point(controlX, controlY, Point.CARTESIAN),  // Curve control
    new Point(endX, endY, Point.CARTESIAN)
));
```

### PathChain
Multiple paths executed in sequence:
```java
PathChain chain = follower.pathBuilder()
    .addPath(path1)
    .addPath(path2)
    .addPath(path3)
    .build();
```

## Testing & Tuning

### Step 1: Test Motor Directions
1. Run autonomous in "test" mode
2. Verify robot drives forward, not backward
3. Adjust motor directions in `PedroPathingConstants.java` if needed

### Step 2: Measure Velocities
1. Create a velocity tuning OpMode (or use Pedro's built-in tuner)
2. Drive robot at full speed in each direction
3. Record max velocities and update constants

### Step 3: Tune PID Values
1. Start with default PID values
2. Run autonomous and observe:
   - Too slow to reach waypoint? → Increase kP
   - Oscillating? → Increase kD
   - Steady-state error? → Add small kI
3. Iterate until smooth path following

### Step 4: Refine Paths
1. Test autonomous sequence
2. Adjust waypoint positions based on field measurements
3. Add/remove waypoints as needed
4. Tune heading interpolation for smooth turns

## Troubleshooting

### Robot doesn't move
- Check motor directions in constants
- Verify Pinpoint odometry is configured correctly
- Check that `follower.update()` is called in loop

### Robot doesn't follow path accurately
- Increase translational PID kP
- Decrease follow distance for tighter following
- Check max velocity constraints

### Robot overshoots waypoints
- Decrease translational PID kP
- Increase translational PID kD
- Increase path end tolerance

### Robot spins unexpectedly
- Check heading PID values
- Verify heading interpolation is set correctly
- Check Pinpoint odometry heading calibration

## Resources

- **Pedro Pathing Documentation**: https://pedropathing.com/docs/
- **Tuning Guide**: https://pedropathing.com/docs/tuning/
- **Examples**: https://pedropathing.com/docs/pathing/examples/
- **Discord Support**: [Pedro Pathing Discord](https://discord.gg/pedropathing)

## Next Steps

1. ✅ Sync Gradle and build project
2. ✅ Configure Pinpoint in hardware config
3. ✅ Measure and set max velocities
4. ✅ Test basic path following
5. ✅ Tune PID values
6. ✅ Customize autonomous sequence
7. ✅ Test on competition field
8. ✅ Iterate and improve!

Good luck with your autonomous! 🤖
