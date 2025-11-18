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
     * Comprehensive shooting presets with flywheel power, target RPM, and hood position
     * Using power-based control for simplicity and reliability
     */
    public enum FlywheelState {
        LONG_RANGE(0.95, 3200, 0.60),    // Long range: 95% power, ~3200 RPM target, high hood
        MEDIUM_RANGE(0.70, 2400, 0.25),  // Medium range: 70% power, ~2400 RPM target, medium hood
        SHORT_RANGE(0.60, 2200, 0.15);   // Short range: 60% power, ~2200 RPM target, low hood

        private final double power;      // Motor power (0.0 to 1.0)
        private final int targetRPM;     // Target RPM for display/reference
        private final double hoodPosition;

        FlywheelState(double power, int targetRPM, double hoodPosition) {
            this.power = power;
            this.targetRPM = targetRPM;
            this.hoodPosition = hoodPosition;
        }

        public double getPower() {
            return power;
        }

        public int getTargetRPM() {
            return targetRPM;
        }

        public double getHoodPosition() {
            return hoodPosition;
        }
    }

    // Hardware
    private final Servo hoodServo;        // Position mode - hood adjustment
    private final DcMotorEx flywheelMotor; // goBilda motor - flywheel shooter

    // State tracking
    private FlywheelState currentState = FlywheelState.SHORT_RANGE; // Default to short range
    private double currentPower = 0.0; // Track current power setting

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

    /**
     * Constructor - only needs HardwareMap
     * @param hardwareMap The hardware map from the OpMode
     */
    public ShooterSubsystem(HardwareMap hardwareMap) {
        // Initialize servos
        hoodServo = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);

        // Initialize motors
        flywheelMotor = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);

        // Configure flywheel motor for direct power control
        // We reset encoder for velocity reading, but use RUN_WITHOUT_ENCODER for direct power
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE); // Reverse motor direction
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Direct power control, no PID
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Coast when stopped

        // Initialize to default positions
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
        power = clamp(power, -1.0, 1.0);
        currentPower = power;
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
     * Run flywheel forward at specified power
     * @param power Power from 0.0 to 1.0 (0% to 100%)
     */
    public void runFlywheelForward(double power) {
        setFlywheelPower(Math.abs(power));
    }

    /**
     * Run flywheel in reverse at specified power
     * @param power Power from 0.0 to 1.0 (0% to 100%)
     */
    public void runFlywheelReverse(double power) {
        setFlywheelPower(-Math.abs(power));
    }

    /**
     * Stop the flywheel
     */
    public void stopFlywheel() {
        currentPower = 0.0;
        flywheelMotor.setPower(0);
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
     * Set the flywheel state (range preset) - sets both flywheel power and hood position
     * @param state The desired flywheel state
     */
    public void setFlywheelState(FlywheelState state) {
        currentState = state;
        setFlywheelPower(state.getPower());
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
     * Check if flywheel has reached approximately its target RPM
     * Uses a tolerance of 10% to account for real-world variance with power control
     * @return true if flywheel is near target RPM
     */
    public boolean isAtTargetRPM() {
        double targetRPM = currentState.getTargetRPM();
        if (targetRPM == 0) {
            return Math.abs(getCurrentRPM()) < 100.0; // Consider stopped if < 100 RPM
        }
        double tolerance = targetRPM * 0.10; // 10% tolerance for power-based control
        double actualRPM = getCurrentRPM();
        return Math.abs(actualRPM - targetRPM) <= tolerance;
    }

    /**
     * Get the target RPM from current preset state
     * @return Target RPM
     */
    public int getTargetRPM() {
        return currentState.getTargetRPM();
    }

    /**
     * Increment flywheel power by 5%
     */
    public void incrementFlywheelPower() {
        setFlywheelPower(currentPower + 0.05);
    }

    /**
     * Decrement flywheel power by 5%
     */
    public void decrementFlywheelPower() {
        setFlywheelPower(currentPower - 0.05);
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