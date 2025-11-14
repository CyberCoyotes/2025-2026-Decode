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
public class ShooterSubsystem {

    /**
     * Flywheel shooting range states with associated RPM values
     */
    public enum FlywheelState {
        LONG_RANGE(3200),    // Long range shot - 3200 RPM
        MEDIUM_RANGE(1800),  // Medium range shot - 3000 RPM
        SHORT_RANGE(1500);   // Short range shot - 2600 RPM

        private final int rpm;

        FlywheelState(int rpm) {
            this.rpm = rpm;
        }

        public int getRPM() {
            return rpm;
        }

        /**
         * Convert RPM to velocity in ticks per second
         * Formula: RPM / 60 * CPR (counts per revolution)
         * @return velocity in ticks per second
         */
        public double getVelocity() {
            return (rpm / 60.0) * 28.0; // 28 CPR for Yellow Jacket motor
        }
    }

    // Hardware
    private final Servo turretServo;      // Position mode - turret rotation
    private final Servo hoodServo;        // Position mode - hood adjustment
    private final DcMotorEx flywheelMotor; // goBilda motor - flywheel shooter

    // State tracking
    private FlywheelState currentState = FlywheelState.MEDIUM_RANGE; // Default to medium range
    private double targetVelocity = 0.0;

    // Hardware configuration names
    private static final String TURRET_SERVO_NAME = "turretServo";
    private static final String HOOD_SERVO_NAME = "hoodServo";
    private static final String FLYWHEEL_MOTOR_NAME = "flywheelMotor";

    // Constants for turret servo positions
    private static final double TURRET_MIN_POSITION = 0.0;
    private static final double TURRET_MAX_POSITION = 1.0;
    private static final double TURRET_CENTER_POSITION = 0.5;

    // Constants for hood servo positions
    private static final double HOOD_MIN_POSITION = 0.1; // TODO Adjust as needed
    private static final double HOOD_MAX_POSITION = 0.9; // TODO Adjust as needed
    private static final double HOOD_DEFAULT_POSITION = 0.5;

    // Constants for flywheel motor (5203 Series Yellow Jacket 6000 RPM 1:1 Ratio)
    // Motor specs: 6000 RPM free speed, 28 CPR (counts per revolution)
    // Max velocity: 6000 RPM / 60 sec = 100 RPS * 28 CPR = 2800 ticks/second
    private static final double FLYWHEEL_MAX_VELOCITY = 2800.0; // ticks per second at 100% power
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

        // Configure flywheel motor for velocity control with encoder
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE); // Reverse motor direction
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
     * Set the flywheel motor power directly
     * @param power Power from -1.0 (full reverse) to 1.0 (full forward)
     */
    public void setFlywheelPower(double power) {
        power = clamp(power, -FLYWHEEL_MAX_POWER, FLYWHEEL_MAX_POWER);
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
     * Run flywheel forward at specified power (percentage-based)
     * @param powerPercentage Power percentage from 0.0 to 1.0 (0% to 100%)
     */
    public void runFlywheelForward(double powerPercentage) {
        double clampedPercentage = clamp(Math.abs(powerPercentage), 0.0, FLYWHEEL_MAX_POWER);
        double targetVelocity = clampedPercentage * FLYWHEEL_MAX_VELOCITY;
        flywheelMotor.setVelocity(targetVelocity);
    }

    /**
     * Run flywheel in reverse at specified power (percentage-based)
     * @param powerPercentage Power percentage from 0.0 to 1.0 (0% to 100%)
     */
    public void runFlywheelReverse(double powerPercentage) {
        double clampedPercentage = clamp(Math.abs(powerPercentage), 0.0, FLYWHEEL_MAX_POWER);
        double targetVelocity = clampedPercentage * FLYWHEEL_MAX_VELOCITY;
        flywheelMotor.setVelocity(-targetVelocity);
    }

    /**
     * Set flywheel to specific velocity in ticks per second
     * @param velocity Target velocity in ticks per second
     */
    public void setFlywheelVelocity(double velocity) {
        flywheelMotor.setVelocity(velocity);
    }

    /**
     * Stop the flywheel
     */
    public void stopFlywheel() {
        targetVelocity = 0.0;
        flywheelMotor.setVelocity(0.0);
    }

    /**
     * Get the current flywheel velocity
     * @return Current velocity in ticks per second
     */
    public double getFlywheelVelocity() {
        return flywheelMotor.getVelocity();
    }

    /**
     * Get the target flywheel velocity
     * @return Target velocity in ticks per second
     */
    public double getFlywheelTargetVelocity() {
        // Note: DcMotorEx doesn't have a direct getTargetVelocity method,
        // so we calculate it from the current power setting
        return getFlywheelPower() * FLYWHEEL_MAX_VELOCITY;
    }

    /**
     * Get the maximum flywheel velocity
     * @return Max velocity in ticks per second
     */
    public double getFlywheelMaxVelocity() {
        return FLYWHEEL_MAX_VELOCITY;
    }

    /**
     * Get current flywheel velocity as a percentage of max
     * @return Velocity percentage from 0.0 to 1.0
     */
    public double getFlywheelVelocityPercentage() {
        return getFlywheelVelocity() / FLYWHEEL_MAX_VELOCITY;
    }

    /**
     * Set the flywheel state (range preset)
     * @param state The desired flywheel state
     */
    public void setFlywheelState(FlywheelState state) {
        currentState = state;
        targetVelocity = state.getVelocity();
        flywheelMotor.setVelocity(targetVelocity);
    }

    /**
     * Get the current flywheel state
     * @return Current flywheel state
     */
    public FlywheelState getFlywheelState() {
        return currentState;
    }

    /**
     * Check if flywheel has reached its target velocity
     * Uses a tolerance of 5% to account for real-world variance
     * @return true if flywheel is at target velocity
     */
    public boolean isAtTargetVelocity() {
        if (targetVelocity == 0.0) {
            return Math.abs(getFlywheelVelocity()) < 50.0; // Consider stopped if < 50 tps
        }
        double tolerance = targetVelocity * 0.05; // 5% tolerance
        double currentVelocity = getFlywheelVelocity();
        return Math.abs(currentVelocity - targetVelocity) <= tolerance;
    }

    /**
     * Get the current target velocity
     * @return Target velocity in ticks per second
     */
    public double getTargetVelocity() {
        return targetVelocity;
    }

    /**
     * Get the target RPM based on current state
     * @return Target RPM
     */
    public int getTargetRPM() {
        return currentState.getRPM();
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