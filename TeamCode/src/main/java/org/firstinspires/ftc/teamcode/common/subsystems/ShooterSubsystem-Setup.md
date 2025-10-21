# ScoringSubsystem Hardware Configuration Guide

## Overview
The ScoringSubsystem controls three components for shooting game elements:
1. **Turret Servo** - Rotates the shooter horizontally (position mode)
2. **Hood Servo** - Adjusts the shooting angle (position mode)
3. **Flywheel Motor** - Spins the flywheel to launch game elements (goBilda motor)

## Hardware Requirements
- 1x goBilda Position Servo (for turret rotation)
- 1x goBilda Position Servo (for hood adjustment)
- 1x goBilda Motor (for flywheel shooter)

## Robot Configuration Setup

### Step 1: Connect Hardware to Control Hub
1. Connect the turret servo to a servo port on the Control Hub
2. Connect the hood servo to a servo port on the Control Hub
3. Connect the flywheel motor to a motor port on the Control Hub

### Step 2: Configure in FTC Robot Controller App
1. Open the FTC Robot Controller app
2. Go to "Configure Robot"
3. Select your active configuration or create a new one
4. Add the following devices with **exact names** (case-sensitive):

#### Turret Servo Configuration
- **Type**: Servo
- **Name**: `turretServo`
- **Port**: (the port where you connected the turret servo)

#### Hood Servo Configuration
- **Type**: Servo
- **Name**: `hoodServo`
- **Port**: (the port where you connected the hood servo)

#### Flywheel Motor Configuration
- **Type**: Motor (goBilda/REV motor)
- **Name**: `flywheelMotor`
- **Port**: (the port where you connected the flywheel motor)
- **Direction**: Test and adjust as needed (may need to reverse)

### Step 3: Save Configuration
1. Save your robot configuration
2. Make sure to activate this configuration before running your OpMode

## Usage in OpMode

### Initialization

```java
import org.firstinspires.ftc.teamcode.common.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.ShooterSubsystem;

// In your OpMode
private ShooterSubsystem shooterSubsystem;

        @Override
        public void runOpMode() {
            // Initialize the subsystem
            scoringSubsystem = new ShooterSubsystem(hardwareMap);

            waitForStart();

            // Your code here
        }
```

### Basic Control Examples

#### Control Turret Position
```java
// Set turret to center
scoringSubsystem.centerTurret();

// Set turret to custom position (0.0 to 1.0)
scoringSubsystem.setTurretPosition(0.7);

// Get current turret position
double turretPos = scoringSubsystem.getTurretPosition();
```

#### Control Hood Position
```java
// Set hood to down position
scoringSubsystem.hoodDown();

// Set hood to up position
scoringSubsystem.hoodUp();

// Set hood to default position
scoringSubsystem.hoodDefault();

// Set hood to custom position (0.0 to 1.0)
scoringSubsystem.setHoodPosition(0.5);

// Get current hood position
double hoodPos = scoringSubsystem.getHoodPosition();
```

#### Control Flywheel
```java
// Run flywheel at full speed
scoringSubsystem.runFlywheel();

// Run flywheel at specific power (0.0 to 1.0)
scoringSubsystem.runFlywheel(0.8);

// Set flywheel power directly (-1.0 to 1.0)
scoringSubsystem.setFlywheelPower(0.75);

// Stop flywheel
scoringSubsystem.stopFlywheel();

// Get current flywheel power
double power = scoringSubsystem.getFlywheelPower();

// Get flywheel velocity (if using encoder)
double velocity = scoringSubsystem.getFlywheelVelocity();
```

#### Combined Control
```java
// Reset all to default positions
scoringSubsystem.reset();

// Stop all motors (servos maintain position)
scoringSubsystem.stopAll();
```

### Example TeleOp Integration
```java
// In your main control loop
while (opModeIsActive()) {
    // Turret control with left stick X
    if (Math.abs(gamepad2.left_stick_x) > 0.1) {
        double turretPos = scoringSubsystem.getTurretPosition();
        turretPos += gamepad2.left_stick_x * 0.01; // Adjust speed as needed
        scoringSubsystem.setTurretPosition(turretPos);
    }
    
    // Hood control with right stick Y
    if (Math.abs(gamepad2.right_stick_y) > 0.1) {
        double hoodPos = scoringSubsystem.getHoodPosition();
        hoodPos -= gamepad2.right_stick_y * 0.01; // Adjust speed as needed
        scoringSubsystem.setHoodPosition(hoodPos);
    }
    
    // Flywheel control with triggers
    if (gamepad2.right_trigger > 0.1) {
        scoringSubsystem.runFlywheel(gamepad2.right_trigger);
    } else {
        scoringSubsystem.stopFlywheel();
    }
    
    // Quick positions with buttons
    if (gamepad2.a) {
        scoringSubsystem.reset();
    }
    if (gamepad2.b) {
        scoringSubsystem.centerTurret();
    }
    if (gamepad2.x) {
        scoringSubsystem.hoodDefault();
    }
    
    // Telemetry
    telemetry.addData("Turret Position", "%.2f", scoringSubsystem.getTurretPosition());
    telemetry.addData("Hood Position", "%.2f", scoringSubsystem.getHoodPosition());
    telemetry.addData("Flywheel Power", "%.2f", scoringSubsystem.getFlywheelPower());
    telemetry.update();
}

// Cleanup
scoringSubsystem.stopAll();
```

## Tuning and Calibration

### Position Constants
The subsystem includes default position constants that may need adjustment for your robot:

In `ScoringSubsystem.java`:
```java
// Turret positions
private static final double TURRET_CENTER_POSITION = 0.5;
private static final double TURRET_MIN_POSITION = 0.0;
private static final double TURRET_MAX_POSITION = 1.0;

// Hood positions
private static final double HOOD_DOWN_POSITION = 0.0;
private static final double HOOD_UP_POSITION = 1.0;
private static final double HOOD_DEFAULT_POSITION = 0.3;
```

### Calibration Steps
1. Test each servo's range of motion
2. Identify the minimum and maximum safe positions
3. Update the constants in the code if needed
4. Test the flywheel motor direction
5. Adjust motor direction in robot config if it spins backwards

## Troubleshooting

### Servos not moving
- Check that servo names in robot config match exactly: `turretServo`, `hoodServo`
- Verify servos are connected to correct ports
- Check that robot configuration is saved and activated

### Motor not spinning
- Check that motor name in robot config matches: `flywheelMotor`
- Verify motor is connected to correct port
- Check motor direction setting in robot config
- Test with `setFlywheelPower(1.0)` to verify motor works

### Servos moving in wrong direction
- Adjust position values in the code
- Or invert the servo direction in the robot configuration

### Motor spinning in wrong direction
- Change motor direction in robot configuration (Reverse/Forward)
- Or modify the code to invert power values

## Additional Resources
- [Programming and Implementing Servos in FTC - goBILDA](https://youtu.be/s_4Xvi8vru4?si=Jtw7sTyKQFPNEQeA)
- [FTC Documentation](https://ftc-docs.firstinspires.org/)
- [goBilda Documentation](https://www.gobilda.com/)
