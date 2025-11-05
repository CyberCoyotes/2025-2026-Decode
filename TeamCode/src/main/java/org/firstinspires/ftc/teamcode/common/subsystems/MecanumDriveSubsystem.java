package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mecanum Drive Subsystem for Teams 11940 & Teams 22091
 * Supports field-centric and robot-centric drive modes
 * Configurable speed, sensitivity, and deadzone
 * Implements state machine for different drive modes
 */
public class MecanumDriveSubsystem {

    /* ========================================
     * DRIVE STATE ENUM
     * ======================================== */
    public enum DriveState {
        IDLE,              // Emergency stop - no movement
        ROBOT_CENTRIC,     // Normal robot-relative driving
        FIELD_CENTRIC,     // Field-relative driving using IMU
        PRECISION,         // Slow, precise movements (50% speed)
        TURBO              // Fast movements (100% speed)
    }

    /* ========================================
     * HARDWARE
     * ======================================== */
    private final DcMotorEx leftFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightBack;
    private final IMU imu;

    /* ========================================
     * STATE MACHINE
     * ======================================== */
    private DriveState currentState = DriveState.FIELD_CENTRIC;  // Default to field-centric for competition

    /* ========================================
     * CONFIGURATION VARIABLES
     * ======================================== */
    private double speedMultiplier = 1.0;      // Overall speed (0.0 to 1.0)
    private double sensitivityMultiplier = 1.0; // Sensitivity curve (0.5 to 2.0)
    private double deadzone = 0.1;              // Joystick deadzone (0.0 to 0.3)
    private boolean fieldCentric = true;        // Enable/disable field-centric drive

    /* ========================================
     * CONSTANTS
     * ======================================== */
    private static final double MIN_SPEED = 0.1;
    private static final double MAX_SPEED = 1.0;
    private static final double MIN_SENSITIVITY = 0.5;
    private static final double MAX_SENSITIVITY = 2.0;
    private static final double MIN_DEADZONE = 0.0;
    private static final double MAX_DEADZONE = 0.3;

    /* ========================================
     * CONSTRUCTOR
     * ======================================== */
    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        // Initialize motors with hardware map names
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Set motor directions
        // TODO: Test and adjust these directions based on your robot's wiring
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior to brake for better control
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motors to run without encoders for teleop
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Configure IMU orientation based on Control Hub mounting
        // COMPETITION BOT MOUNTING CONFIGURATION:
        // - Control Hub is mounted VERTICALLY on the LEFT side of the robot
        // - Located approximately 7 inches toward the back
        // - Located approximately 7 inches to the left
        // - Logo/stickers face INWARD (toward the center/right side of robot)
        // - USB ports face UP
        //
        // This configuration is CRITICAL for field-centric driving to work correctly!
        //
        // To verify orientation is correct, test that:
        // - Forward on stick moves robot FORWARD (in field-centric mode)
        // - Yaw increases when robot rotates COUNTER-CLOCKWISE (✓ verified working)
        // - Pitch increases when robot front tips UP
        //
        // For vertically mounted Control Hub on left side with logo facing right/inward:
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,  // Logo faces inward (right/center)
                RevHubOrientationOnRobot.UsbFacingDirection.UP       // USB ports face up
        ));
        imu.initialize(parameters);
    }

    /* ========================================
     * DRIVE METHODS
     * ======================================== */

    /**
     * Main drive method for mecanum wheels
     * Supports both field-centric and robot-centric modes
     * Behavior changes based on current state
     *
     * @param axial Forward/backward motion (left stick Y)
     * @param lateral Left/right strafe motion (left stick X)
     * @param yaw Rotation motion (right stick X)
     */
    public void drive(double axial, double lateral, double yaw) {
        // Handle IDLE state - stop all motors
        if (currentState == DriveState.IDLE) {
            stop();
            return;
        }

        // Apply deadzone first
        axial = applyDeadzone(axial);
        lateral = applyDeadzone(lateral);
        yaw = applyDeadzone(yaw);

        // Apply sensitivity curve
        axial = applySensitivity(axial);
        lateral = applySensitivity(lateral);
        yaw = applySensitivity(yaw);

        // Apply field-centric transformation if in FIELD_CENTRIC state
        if (currentState == DriveState.FIELD_CENTRIC) {
            // Get the robot's current heading from the IMU
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction to be relative to the field
            // This uses a 2D rotation matrix to transform joystick inputs
            double rotatedAxial = axial * Math.cos(-botHeading) - lateral * Math.sin(-botHeading);
            double rotatedLateral = axial * Math.sin(-botHeading) + lateral * Math.cos(-botHeading);

            axial = rotatedAxial;
            lateral = rotatedLateral;
        }

        // Calculate wheel powers using mecanum drive kinematics
        double leftFrontPower = axial + lateral + yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize wheel powers to ensure no value exceeds 1.0
        double maxPower = Math.max(
                Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower)),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))
        );

        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightFrontPower /= maxPower;
            rightBackPower /= maxPower;
        }

        // Apply state-based speed multiplier and base speed multiplier
        double effectiveSpeed = speedMultiplier * getStateSpeedMultiplier();
        leftFrontPower *= effectiveSpeed;
        leftBackPower *= effectiveSpeed;
        rightFrontPower *= effectiveSpeed;
        rightBackPower *= effectiveSpeed;

        // Set motor powers
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    /**
     * Stop all drive motors
     */
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    /**
     * Reset the IMU yaw to zero
     * This defines the current heading as "forward" for field-centric mode
     * Call this at the start of TeleOp or when repositioning the robot
     */
    public void resetHeading() {
        imu.resetYaw();
    }

    /* ========================================
     * CONFIGURATION METHODS
     * ======================================== */

    /**
     * Enable or disable field-centric drive
     * @param enabled true for field-centric, false for robot-centric
     * @deprecated Use setState() instead for better state control
     */
    @Deprecated
    public void setFieldCentric(boolean enabled) {
        this.fieldCentric = enabled;
        // Also update state for compatibility
        if (enabled) {
            setState(DriveState.FIELD_CENTRIC);
        } else {
            setState(DriveState.ROBOT_CENTRIC);
        }
    }

    /**
     * Toggle between field-centric and robot-centric modes
     * @return The new state (true = field-centric, false = robot-centric)
     * @deprecated Use setState() instead for better state control
     */
    @Deprecated
    public boolean toggleFieldCentric() {
        this.fieldCentric = !this.fieldCentric;
        // Also update state for compatibility
        if (fieldCentric) {
            setState(DriveState.FIELD_CENTRIC);
        } else {
            setState(DriveState.ROBOT_CENTRIC);
        }
        return this.fieldCentric;
    }

    /**
     * Set the overall speed multiplier
     * @param speed Speed multiplier (0.1 to 1.0)
     */
    public void setSpeed(double speed) {
        speedMultiplier = clamp(speed, MIN_SPEED, MAX_SPEED);
    }

    /**
     * Set the sensitivity curve
     * @param sensitivity Sensitivity multiplier (0.5 to 2.0)
     *                   < 1.0 = less sensitive (more gradual)
     *                   = 1.0 = linear
     *                   > 1.0 = more sensitive (more aggressive)
     */
    public void setSensitivity(double sensitivity) {
        sensitivityMultiplier = clamp(sensitivity, MIN_SENSITIVITY, MAX_SENSITIVITY);
    }

    /**
     * Set the joystick deadzone
     * @param deadzone Deadzone threshold (0.0 to 0.3)
     */
    public void setDeadzone(double deadzone) {
        this.deadzone = clamp(deadzone, MIN_DEADZONE, MAX_DEADZONE);
    }

    /**
     * Increase speed by 10%
     */
    public void increaseSpeed() {
        setSpeed(speedMultiplier + 0.1);
    }

    /**
     * Decrease speed by 10%
     */
    public void decreaseSpeed() {
        setSpeed(speedMultiplier - 0.1);
    }

    /**
     * Increase sensitivity
     */
    public void increaseSensitivity() {
        setSensitivity(sensitivityMultiplier + 0.1);
    }

    /**
     * Decrease sensitivity
     */
    public void decreaseSensitivity() {
        setSensitivity(sensitivityMultiplier - 0.1);
    }

    /* ========================================
     * GETTERS
     * ======================================== */

    public double getSpeed() {
        return speedMultiplier;
    }

    public double getSensitivity() {
        return sensitivityMultiplier;
    }

    public double getDeadzone() {
        return deadzone;
    }

    /**
     * Check if currently in field-centric mode
     * @return true if in field-centric state
     */
    public boolean isFieldCentric() {
        return currentState == DriveState.FIELD_CENTRIC;
    }

    /**
     * Get the current robot heading from the IMU
     * @return Heading in degrees (-180 to +180)
     */
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getLeftFrontPower() {
        return leftFront.getPower();
    }

    public double getLeftBackPower() {
        return leftBack.getPower();
    }

    public double getRightFrontPower() {
        return rightFront.getPower();
    }

    public double getRightBackPower() {
        return rightBack.getPower();
    }

    /* ========================================
     * STATE MACHINE METHODS
     * ======================================== */

    /**
     * Set the current drive state
     * @param newState The new state to transition to
     */
    public void setState(DriveState newState) {
        this.currentState = newState;
    }

    /**
     * Get the current drive state
     * @return The current state
     */
    public DriveState getState() {
        return currentState;
    }

    /**
     * Get a string representation of the current state
     * @return State name as string
     */
    public String getStateString() {
        switch (currentState) {
            case IDLE:
                return "IDLE (Stopped)";
            case ROBOT_CENTRIC:
                return "Robot-Centric";
            case FIELD_CENTRIC:
                return "Field-Centric";
            case PRECISION:
                return "Precision Mode";
            case TURBO:
                return "Turbo Mode";
            default:
                return "Unknown";
        }
    }

    /**
     * Get the speed multiplier for the current state
     * @return Speed multiplier (0.0 to 1.0)
     */
    public double getStateSpeedMultiplier() {
        switch (currentState) {
            case IDLE:
                return 0.0;      // No movement
            case ROBOT_CENTRIC:
            case FIELD_CENTRIC:
                return 1.0;      // Normal speed (uses base speed multiplier)
            case PRECISION:
                return 0.5;      // 50% speed for precise movements
            case TURBO:
                return 1.0;      // 100% speed (ignores base speed limit)
            default:
                return 1.0;
        }
    }

    /* ========================================
     * HELPER METHODS
     * ======================================== */

    /**
     * Apply deadzone to input value
     * @param value Raw input value
     * @return Value with deadzone applied (0 if within deadzone)
     */
    private double applyDeadzone(double value) {
        return Math.abs(value) > deadzone ? value : 0.0;
    }

    /**
     * Apply sensitivity curve to input value
     * @param value Input value after deadzone
     * @return Value with sensitivity curve applied
     */
    private double applySensitivity(double value) {
        if (value == 0.0) return 0.0;

        // Preserve sign
        double sign = Math.signum(value);
        double magnitude = Math.abs(value);

        // Apply power curve based on sensitivity
        // sensitivity < 1.0: makes controls more gradual
        // sensitivity = 1.0: linear (no change)
        // sensitivity > 1.0: makes controls more aggressive
        magnitude = Math.pow(magnitude, sensitivityMultiplier);

        return sign * magnitude;
    }

    /**
     * Clamp value between min and max
     * @param value Value to clamp
     * @param min Minimum value
     * @param max Maximum value
     * @return Clamped value
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /* ========================================
     * TELEMETRY HELPER
     * ======================================== */

    /**
     * Get formatted telemetry string for drive subsystem
     * @return Telemetry string with all relevant info
     */
    public String getTelemetry() {
        return String.format(
                "State: %s | Speed Mult: %.0f%%\n" +
                        "Base Speed: %.0f%% | Sensitivity: %.1fx | Deadzone: %.2f\n" +
                        "Heading: %.1f°\n" +
                        "LF: %.2f | RF: %.2f\n" +
                        "LB: %.2f | RB: %.2f",
                getStateString(),
                getStateSpeedMultiplier() * 100,
                speedMultiplier * 100,
                sensitivityMultiplier,
                deadzone,
                getHeading(),
                leftFront.getPower(),
                rightFront.getPower(),
                leftBack.getPower(),
                rightBack.getPower()
        );
    }
}