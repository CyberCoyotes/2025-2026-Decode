package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Components of the Scoring Subsystem:
 * - Hood Adjustment with a goBilda position based servo
 * - Ball launcher-flywheel with a goBilda motor
 */
public class ShooterSubsystem {

    /**
     * Comprehensive shooting presets with flywheel target RPM and hood position
     * Using velocity-based control for consistent speed under varying loads
     */
    public enum FlywheelState {
        LONG_RANGE(2800, 0.60),    // Long range: 2800 RPM target, high hood
        MEDIUM_RANGE(2500, 0.25),  // Medium range: 2500 RPM target, medium hood
        SHORT_RANGE(2200, 0.15);   // Short range: 2200 RPM target, low hood

        private final int targetRPM;     // Target RPM
        private final double hoodPosition;

        FlywheelState(int targetRPM, double hoodPosition) {
            this.targetRPM = targetRPM;
            this.hoodPosition = hoodPosition;
        }

        public int getTargetRPM() {
            return targetRPM;
        }

        public double getHoodPosition() {
            return hoodPosition;
        }

        /**
         * Convert RPM to velocity in ticks per second
         */
        public double getVelocity() {
            return (targetRPM / 60.0) * 28.0; // 28 CPR for Yellow Jacket motor
        }
    }

    // Hardware
    private final Servo hoodServo;        // Position mode - hood adjustment
    private final DcMotorEx flywheelMotor; // goBilda motor - flywheel shooter

    // State tracking
    private FlywheelState currentState = FlywheelState.SHORT_RANGE; // Default to short range
    private double targetVelocity = 0.0; // Track current target velocity in ticks/sec

    // Hardware configuration names
    private static final String HOOD_SERVO_NAME = "hoodServo";
    private static final String FLYWHEEL_MOTOR_NAME = "flywheelMotor";

    // Constants for hood servo positions
    private static final double HOOD_MIN_POSITION = 0.0; //
    private static final double HOOD_MAX_POSITION = 0.60; //
    // Default position is ZERO and the hood should be resting on the hard stop as low as possible.
    private static final double HOOD_DEFAULT_POSITION = 0.0;

    // Constants for flywheel motor (5203 Series Yellow Jacket 6000 RPM 1:1 Ratio)
    // Motor specs: 6000 RPM free speed, 28 CPR (counts per revolution)
    private static final double FLYWHEEL_CPR = 28.0; // Counts per revolution for RPM calculation

    // Simple PIDF coefficients for velocity control
    // Tuned to reach target RPM without overshoot
    private static final double FLYWHEEL_P = 5.0;   // Proportional gain - reduced to prevent overshoot
    private static final double FLYWHEEL_I = 0.1;   // Integral gain - reduced to prevent accumulation
    private static final double FLYWHEEL_D = 1.0;   // Derivative - dampens overshoot and oscillation
    private static final double FLYWHEEL_F = 20.0;  // Feedforward - further reduced to prevent overshoot

    /**
     * Constructor - only needs HardwareMap
     * @param hardwareMap The hardware map from the OpMode
     */
    public ShooterSubsystem(HardwareMap hardwareMap) {
        // Initialize servos
        hoodServo = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);

        // Initialize motors
        flywheelMotor = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);

        // Configure flywheel motor for velocity control with encoder
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Configure simple PIDF for velocity control
        flywheelMotor.setVelocityPIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);

        // Initialize to default positions
        hoodServo.setPosition(HOOD_DEFAULT_POSITION);
        flywheelMotor.setVelocity(0);
    }

    /**
     * Periodic method called once per scheduler run
     */
    public void periodic() {
        // Add any periodic updates here if needed
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
     * Set flywheel velocity directly in ticks per second
     * @param velocity Target velocity in ticks per second
     */
    public void setFlywheelVelocity(double velocity) {
        targetVelocity = velocity;
        flywheelMotor.setVelocity(velocity);
    }

    /**
     * Set flywheel to target RPM
     * @param rpm Target RPM
     */
    public void setFlywheelRPM(int rpm) {
        double velocity = (rpm / 60.0) * FLYWHEEL_CPR;
        setFlywheelVelocity(velocity);
    }

    /**
     * Stop the flywheel
     */
    public void stopFlywheel() {
        targetVelocity = 0.0;
        flywheelMotor.setVelocity(0);
    }

    /**
     * Get the current flywheel power (for telemetry)
     * @return Current power (-1.0 to 1.0)
     */
    public double getFlywheelPower() {
        return flywheelMotor.getPower();
    }

    /**
     * Get the current flywheel velocity in ticks per second
     * @return Current velocity in ticks per second (from encoder)
     */
    public double getFlywheelVelocity() {
        return flywheelMotor.getVelocity();
    }

    /**
     * Get the current flywheel RPM
     * @return Current RPM calculated from encoder velocity
     */
    public double getCurrentRPM() {
        return (getFlywheelVelocity() / FLYWHEEL_CPR) * 60.0;
    }

    /**
     * Set the flywheel state (range preset) - sets both flywheel velocity and hood position
     * @param state The desired flywheel state
     */
    public void setFlywheelState(FlywheelState state) {
        currentState = state;
        setFlywheelVelocity(state.getVelocity());
        setHoodPosition(state.getHoodPosition());
    }

    /**
     * Get the current flywheel state
     * @return Current flywheel state
     */
    public FlywheelState getFlywheelState() {
        return currentState;
    }

    /**
     * Check if flywheel has reached approximately its target velocity
     * Uses a tolerance of 5% to account for real-world variance
     * @return true if flywheel is near target velocity
     */
    public boolean isAtTargetRPM() {
        if (targetVelocity == 0.0) {
            return Math.abs(getFlywheelVelocity()) < 50.0; // Consider stopped if < 50 ticks/sec
        }
        double tolerance = targetVelocity * 0.05; // 5% tolerance
        double actualVelocity = getFlywheelVelocity();
        return Math.abs(actualVelocity - targetVelocity) <= tolerance;
    }

    /**
     * Get the target RPM from current preset state
     * @return Target RPM
     */
    public int getTargetRPM() {
        return currentState.getTargetRPM();
    }

    /**
     * Get the target velocity in ticks per second
     * @return Target velocity
     */
    public double getTargetVelocity() {
        return targetVelocity;
    }

    /**
     * Increment flywheel RPM by 100
     */
    public void incrementFlywheelRPM() {
        int currentRPM = (int)getCurrentRPM();
        setFlywheelRPM(currentRPM + 100);
    }

    /**
     * Decrement flywheel RPM by 100
     */
    public void decrementFlywheelRPM() {
        int currentRPM = (int)getCurrentRPM();
        setFlywheelRPM(currentRPM - 100);
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