package org.firstinspires.ftc.teamcode.practicebot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Components of the Scoring Subsystem:
 * - Turret rotation with a goBilda position based servo
 * - Hood Adjustment with a goBilda position based servo
 * - Ball launcher-flywheel with a goBilda motor
 */
public class ShooterSubsystem {

    // Hardware
    private final Servo turretServo;      // Position mode - turret rotation
    private final Servo hoodServo;        // Position mode - hood adjustment
    private final DcMotorEx flywheelMotor; // goBilda motor - flywheel shooter

    // Hardware configuration names
    private static final String TURRET_SERVO_NAME = "turretServo";
    private static final String HOOD_SERVO_NAME = "hoodServo";
    private static final String FLYWHEEL_MOTOR_NAME = "flywheelMotor";
    private static final String LEFT_FRONT_MOTOR_NAME = "leftFront";

    // Constants for turret servo positions
    private static final double TURRET_MIN_POSITION = 0.0;
    private static final double TURRET_MAX_POSITION = 1.0;
    private static final double TURRET_CENTER_POSITION = 0.5;

    // Constants for hood servo positions
    private static final double HOOD_MIN_POSITION = 0.1; // TODO Adjust as needed
    private static final double HOOD_MAX_POSITION = 0.9; // TODO Adjust as needed
    private static final double HOOD_DEFAULT_POSITION = 0.5;

    // Constants for flywheel motor
    private static final double FLYWHEEL_MIN_POWER = 0.0;
    private static final double FLYWHEEL_MAX_POWER = 1.0;

    /**
     * Constructor - only needs HardwareMap
     * @param hardwareMap The hardware map from the OpMode
     */
    public ShooterSubsystem(HardwareMap hardwareMap) {
        // Initialize servos
        turretServo = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);
        hoodServo = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);

        // Initialize motors
        flywheelMotor = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);

        // Configure flywheel motor
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // Initialize to default positions
        turretServo.setPosition(TURRET_CENTER_POSITION);
        hoodServo.setPosition(HOOD_DEFAULT_POSITION);
        flywheelMotor.setPower(0);
    }

    /**
     * Periodic method called once per scheduler run
     */
    public void periodic() {
        // Add any periodic updates here if needed
    }

    /* ========================================
     * TURRET CONTROL METHODS
     * ======================================== */

    /**
     * Set the turret servo position
     * @param position Position from 0.0 to 1.0
     */
    public void setTurretPosition(double position) {
        position = clamp(position, TURRET_MIN_POSITION, TURRET_MAX_POSITION);
        turretServo.setPosition(position);
    }

    /**
     * Center the turret
     */
    public void centerTurret() {
        turretServo.setPosition(TURRET_CENTER_POSITION);
    }

    /**
     * Get the current turret position
     * @return Current position (0.0 to 1.0)
     */
    public double getTurretPosition() {
        return turretServo.getPosition();
    }

    /* ========================================
     * HOOD CONTROL METHODS
     * ======================================== */

    /**
     * Set the hood servo position
     * @param position Position from 0.0 (min) to 1.0 (max)
     */
    public void setHoodPosition(double position) {
        position = clamp(position, HOOD_MIN_POSITION, HOOD_MAX_POSITION);
        hoodServo.setPosition(position);
    }

    /**
     * Get the current hood position
     * @return Current position (0.0 to 1.0)
     */
    public double getHoodPosition() {
        return hoodServo.getPosition();
    }

    /**
     * Set hood to default position
     */
    public void hoodDefault() {
        setHoodPosition(HOOD_DEFAULT_POSITION);
    }

    /* ========================================
     * FLYWHEEL CONTROL METHODS
     * ======================================== */

    /**
     * Set the flywheel motor power
     * @param power Power from 0.0 (stopped) to 1.0 (full speed)
     */
    public void setFlywheelPower(double power) {
        power = clamp(power, FLYWHEEL_MIN_POWER, FLYWHEEL_MAX_POWER);
        flywheelMotor.setPower(power);
    }

    /**
     * Get the current flywheel power
     * @return Current power (0.0 to 1.0)
     */
    public double getFlywheelPower() {
        return flywheelMotor.getPower();
    }

    /**
     * Run flywheel at full speed
     */
    public void runFlywheel() {
        setFlywheelPower(FLYWHEEL_MAX_POWER);
    }

    /**
     * Run flywheel at specific power
     * @param power Power from 0.0 to 1.0
     */
    public void runFlywheel(double power) {
        setFlywheelPower(-Math.abs(power)); // Negative sign to match correct direction

    }

    public void runFlywheelReverse(double power) {
        flywheelMotor.setPower(Math.abs(power)); // Positive sign to match correct direction
    }

    /**
     * Stop the flywheel
     */
    public void stopFlywheel() {
        flywheelMotor.setPower(0.0);
    }

    /**
     * Get the current flywheel velocity if using encoder
     * @return Velocity in ticks per second
     */
    public double getFlywheelVelocity() {
        return flywheelMotor.getVelocity();
    }



    /* ========================================
     * COMBINED CONTROL METHODS
     * ======================================== */

    /**
     * Stop all scoring subsystem components
     */
    public void stopAll() {
        stopFlywheel();
        // Servos maintain their last position
    }

    /**
     * Reset to default positions and stop motors
     */
    public void reset() {
        centerTurret();
        hoodDefault();
        stopFlywheel();
    }

    /* ========================================
     * HELPER METHODS
     * ======================================== */

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

}