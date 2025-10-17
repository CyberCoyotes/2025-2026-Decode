package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Mecanum Drive Subsystem for Teams 11940 & Teams 22091
 * Supports configurable speed, sensitivity, and deadzone
 */
public class MecanumDriveSubsystem {

    /* ========================================
     * HARDWARE
     * ======================================== */
    private final DcMotorEx leftFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightBack;

    /* ========================================
     * CONFIGURATION VARIABLES
     * ======================================== */
    private double speedMultiplier = 1.0;      // Overall speed (0.0 to 1.0)
    private double sensitivityMultiplier = 1.0; // Sensitivity curve (0.5 to 2.0)
    private double deadzone = 0.1;              // Joystick deadzone (0.0 to 0.3)

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
        // TODO: Verify these names match your robot configuration!
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
    }

    /* ========================================
     * DRIVE METHODS
     * ======================================== */

    /**
     * Main drive method for mecanum wheels
     * @param axial Forward/backward motion (left stick Y)
     * @param lateral Left/right strafe motion (left stick X)
     * @param yaw Rotation motion (right stick X)
     */
    public void drive(double axial, double lateral, double yaw) {
        // Apply deadzone
        axial = applyDeadzone(axial);
        lateral = applyDeadzone(lateral);
        yaw = applyDeadzone(yaw);

        // Apply sensitivity curve
        axial = applySensitivity(axial);
        lateral = applySensitivity(lateral);
        yaw = applySensitivity(yaw);

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

        // Apply speed multiplier
        leftFrontPower *= speedMultiplier;
        leftBackPower *= speedMultiplier;
        rightFrontPower *= speedMultiplier;
        rightBackPower *= speedMultiplier;

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

    /* ========================================
     * CONFIGURATION METHODS
     * ======================================== */

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
                "Speed: %.0f%% | Sensitivity: %.1fx | Deadzone: %.2f\n" +
                        "LF: %.2f | RF: %.2f\n" +
                        "LB: %.2f | RB: %.2f",
                speedMultiplier * 100,
                sensitivityMultiplier,
                deadzone,
                leftFront.getPower(),
                rightFront.getPower(),
                leftBack.getPower(),
                rightBack.getPower()
        );
    }
}