package org.firstinspires.ftc.teamcode.common.subsystems;

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

public class ScoringSubsystem {

    // Hardware
    private final Servo turretServo;      // Position mode - turret rotation
    private final Servo hoodServo;        // Position mode - hood adjustment
    private final DcMotorEx flywheelMotor; // goBilda motor - flywheel shooter

    // Hardware configuration names - hardcoded here!
    private static final String TURRET_SERVO_NAME = "turretServo";
    private static final String HOOD_SERVO_NAME = "hoodServo";
    private static final String FLYWHEEL_MOTOR_NAME = "flywheelMotor";

    // Constants for turret positions (adjust based on your robot's configuration)
    private static final double TURRET_CENTER_POSITION = 0.5;
    private static final double TURRET_MIN_POSITION = 0.0;
    private static final double TURRET_MAX_POSITION = 1.0;

    // Constants for hood positions (adjust based on your robot's configuration)
    private static final double HOOD_DOWN_POSITION = 0.0;
    private static final double HOOD_UP_POSITION = 1.0;
    private static final double HOOD_DEFAULT_POSITION = 0.3;

    /**
     * Constructor - only needs HardwareMap
     * @param hardwareMap The hardware map from the OpMode
     */
    public ScoringSubsystem(HardwareMap hardwareMap) {
        // Initialize servos
        turretServo = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);
        hoodServo = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);
        
        // Initialize motor
        flywheelMotor = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize to safe positions
        turretServo.setPosition(TURRET_CENTER_POSITION);
        hoodServo.setPosition(HOOD_DEFAULT_POSITION);
        flywheelMotor.setPower(0.0);
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
        // Clamp position to valid range
        position = Math.max(TURRET_MIN_POSITION, Math.min(TURRET_MAX_POSITION, position));
        turretServo.setPosition(position);
    }

    /**
     * Get the current turret position
     * @return Current position (0.0 to 1.0)
     */
    public double getTurretPosition() {
        return turretServo.getPosition();
    }

    /**
     * Center the turret
     */
    public void centerTurret() {
        setTurretPosition(TURRET_CENTER_POSITION);
    }

    /* ========================================
     * HOOD CONTROL METHODS
     * ======================================== */

    /**
     * Set the hood servo position
     * @param position Position from 0.0 (down) to 1.0 (up)
     */
    public void setHoodPosition(double position) {
        // Clamp position to valid range
        position = Math.max(HOOD_DOWN_POSITION, Math.min(HOOD_UP_POSITION, position));
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
     * Set hood to down position
     */
    public void hoodDown() {
        setHoodPosition(HOOD_DOWN_POSITION);
    }

    /**
     * Set hood to up position
     */
    public void hoodUp() {
        setHoodPosition(HOOD_UP_POSITION);
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
     * @param power Power from -1.0 to 1.0 (typically only positive for shooting)
     */
    public void setFlywheelPower(double power) {
        // Clamp power to valid range
        power = Math.max(-1.0, Math.min(1.0, power));
        flywheelMotor.setPower(power);
    }

    /**
     * Get the current flywheel power
     * @return Current power (-1.0 to 1.0)
     */
    public double getFlywheelPower() {
        return flywheelMotor.getPower();
    }

    /**
     * Run flywheel at full speed
     */
    public void runFlywheel() {
        setFlywheelPower(1.0);
    }

    /**
     * Run flywheel at specific power
     * @param power Power from 0.0 to 1.0
     */
    public void runFlywheel(double power) {
        setFlywheelPower(Math.abs(power));
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

} // end of class
