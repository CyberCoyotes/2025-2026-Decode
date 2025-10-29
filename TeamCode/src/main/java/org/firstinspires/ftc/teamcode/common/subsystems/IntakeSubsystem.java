package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Intake subsystem for collecting and ejecting game artifacts
 * Uses continuous rotation servos for wheels and position servos for slides
 * Implements state machine pattern for robust control
 */
public class IntakeSubsystem {

    /**
     * State machine for intake wheel control
     */
    public enum WheelState {
        IDLE,      // Wheels stopped
        INTAKING,  // Wheels running to intake
        EJECTING   // Wheels running to eject
    }

    /**
     * State machine for intake slide position
     */
    public enum SlideState {
        IN,   // Slides retracted
        OUT   // Slides extended
    }

    // Hardware
    private final Servo intakeWheelLeft;  // Continuous rotation
    private final Servo intakeWheelRight; // Continuous rotation
    private final Servo intakeSlideLeft;  // Position servo
    private final Servo intakeSlideRight; // Position servo

    // Hardware configuration names - hardcoded here!
    private static final String WHEEL_LEFT_NAME = "intakeServoLeft";
    private static final String WHEEL_RIGHT_NAME = "intakeServoRight";
    private static final String SLIDE_LEFT_NAME = "intakeSlideLeft";
    private static final String SLIDE_RIGHT_NAME = "intakeSlideRight";

    // Constants for continuous rotation servos (wheels)
    private static final double STOP_POSITION = 0.5;
    private static final double INTAKE_SPEED = -1.0;   // Full speed intake // TODO test sign change
    private static final double EJECT_SPEED = 1.0;   // Full speed eject

    // Constants for position servos (slides)
    private static final double SLIDE_IN_POSITION = 0.0;
    private static final double SLIDE_OUT_POSITION = 0.75; // Started at 1.0

    // Automatic slide control constants
    private static final long WHEEL_STOP_DELAY_MS = 300; // Delay before stopping wheels after slides retract

    // State tracking
    private WheelState wheelState = WheelState.IDLE;
    private SlideState slideState = SlideState.IN;

    // Automatic slide control state
    private boolean autoSlideControlEnabled = true;
    private boolean delayedWheelStop = false; // True when slides retracted but wheels still running
    private long slideRetractTime = 0; // Time when slides were retracted (in milliseconds)


    /**
     * Constructor - initializes hardware and sets initial states
     * @param hardwareMap The hardware map from the OpMode
     */
    public IntakeSubsystem(HardwareMap hardwareMap) {
        // Initialize wheel servos (continuous rotation)
        intakeWheelLeft = hardwareMap.get(Servo.class, WHEEL_LEFT_NAME);
        intakeWheelRight = hardwareMap.get(Servo.class, WHEEL_RIGHT_NAME);

        // Initialize slide servos (position)
        intakeSlideLeft = hardwareMap.get(Servo.class, SLIDE_LEFT_NAME);
        intakeSlideRight = hardwareMap.get(Servo.class, SLIDE_RIGHT_NAME);

        // Reverse right side servos so they mirror left side
        intakeWheelRight.setDirection(Servo.Direction.REVERSE);
        intakeSlideRight.setDirection(Servo.Direction.REVERSE);

        // Initialize to safe starting state
        setWheelState(WheelState.IDLE);
        setSlideState(SlideState.IN);
    }

    /**
     * Periodic method called once per scheduler run
     * Handles automatic slide control based on intake wheel state
     */
    public void periodic() {
        if (!autoSlideControlEnabled) {
            return; // Manual control mode, don't auto-control slides
        }

        // Automatic slide control logic
        long currentTime = System.currentTimeMillis();

        if (delayedWheelStop) {
            // We're in delayed stop mode - wheels running after slides retracted
            long timeSinceRetract = currentTime - slideRetractTime;
            if (timeSinceRetract >= WHEEL_STOP_DELAY_MS) {
                // Delay period is over, actually stop the wheels now
                wheelState = WheelState.IDLE;
                updateWheelHardware();
                delayedWheelStop = false;
            }
        } else if (wheelState == WheelState.INTAKING || wheelState == WheelState.EJECTING) {
            // Intake is active - extend slides immediately
            if (slideState != SlideState.OUT) {
                setSlideState(SlideState.OUT);
            }
        }
    }

    // ==================== WHEEL STATE MACHINE METHODS ====================

    /**
     * Set the wheel state and update hardware accordingly
     * @param newState The desired wheel state
     */
    public void setWheelState(WheelState newState) {
        wheelState = newState;
        updateWheelHardware();
    }

    /**
     * Get the current wheel state
     * @return Current wheel state
     */
    public WheelState getWheelState() {
        return wheelState;
    }

    /**
     * Get wheel state as a string for telemetry
     * @return State name
     */
    public String getWheelStateString() {
        return wheelState.name();
    }

    /**
     * Update wheel servo positions based on current state
     */
    private void updateWheelHardware() {
        double speed;
        switch (wheelState) {
            case INTAKING:
                speed = INTAKE_SPEED;
                break;
            case EJECTING:
                speed = EJECT_SPEED;
                break;
            case IDLE:
            default:
                speed = 0.0;
                break;
        }
        setWheelSpeed(speed);
    }

    /**
     * Set the speed of the intake wheel servos
     * @param speed Speed from -1.0 (eject) to 1.0 (intake), 0.0 is stop
     */
    private void setWheelSpeed(double speed) {
        // Clamp speed to valid range
        speed = Math.max(-1.0, Math.min(1.0, speed));

        // Convert speed (-1.0 to 1.0) to servo position (0.0 to 1.0)
        double position = STOP_POSITION + (speed * 0.5);

        // Both servos get the same position command
        // Right servo is reversed in hardware, so it will spin opposite direction
        intakeWheelLeft.setPosition(position);
        intakeWheelRight.setPosition(position);
    }

    // ==================== SLIDE STATE MACHINE METHODS ====================

    /**
     * Set the slide state and update hardware accordingly
     * @param newState The desired slide state
     */
    public void setSlideState(SlideState newState) {
        slideState = newState;
        updateSlideHardware();
    }

    /**
     * Get the current slide state
     * @return Current slide state
     */
    public SlideState getSlideState() {
        return slideState;
    }

    /**
     * Get slide state as a string for telemetry
     * @return State name
     */
    public String getSlideStateString() {
        return slideState.name();
    }

    /**
     * Update slide servo positions based on current state
     */
    private void updateSlideHardware() {
        double position;
        switch (slideState) {
            case OUT:
                position = SLIDE_OUT_POSITION;
                break;
            case IN:
            default:
                position = SLIDE_IN_POSITION;
                break;
        }
        setSlidePosition(position);
    }

    /**
     * Set the position of the intake slides
     * @param position Position from 0.0 (in) to 1.0 (out)
     */
    private void setSlidePosition(double position) {
        // Clamp position to valid range
        position = Math.max(0.0, Math.min(1.0, position));

        // Both servos get the same position command
        // Right servo is reversed in hardware, so it will mirror left side
        intakeSlideLeft.setPosition(position);
        intakeSlideRight.setPosition(position);
    }

    // ==================== LEGACY COMPATIBILITY METHODS ====================
    // These maintain backward compatibility with existing teleop code

    /**
     * Stop the intake wheels
     * If auto-control is enabled, slides retract immediately and wheels continue for 300ms
     */
    public void stop() {
        if (autoSlideControlEnabled && (wheelState == WheelState.INTAKING || wheelState == WheelState.EJECTING)) {
            // Retract slides immediately
            setSlideState(SlideState.IN);

            // Enter delayed stop mode - wheels will keep running for 300ms
            delayedWheelStop = true;
            slideRetractTime = System.currentTimeMillis();

            // Don't change wheel state yet - let periodic() handle it after delay
        } else {
            // Not in auto mode or already idle - just stop normally
            setWheelState(WheelState.IDLE);
            delayedWheelStop = false;
        }
    }

    /**
     * Run intake to collect artifacts at full speed
     */
    public void intakeArtifact() {
        delayedWheelStop = false; // Cancel any delayed stop
        setWheelState(WheelState.INTAKING);
    }

    /**
     * Run intake in reverse to eject artifacts at full speed
     */
    public void ejectArtifact() {
        delayedWheelStop = false; // Cancel any delayed stop
        setWheelState(WheelState.EJECTING);
    }

    /**
     * Run intake to collect at a specific speed (legacy method)
     * @param speed Speed from 0.0 to 1.0
     */
    public void intakeArtifact(double speed) {
        setWheelState(WheelState.INTAKING);
        // Note: Custom speed not supported in state machine version
        // Always uses INTAKE_SPEED constant
    }

    /**
     * Run intake in reverse at a specific speed (legacy method)
     * @param speed Speed from 0.0 to 1.0
     */
    public void ejectArtifact(double speed) {
        setWheelState(WheelState.EJECTING);
        // Note: Custom speed not supported in state machine version
        // Always uses EJECT_SPEED constant
    }

    // ==================== AUTOMATIC SLIDE CONTROL METHODS ====================

    /**
     * Enable automatic slide control (slides extend/retract with intake)
     */
    public void enableAutoSlideControl() {
        autoSlideControlEnabled = true;
    }

    /**
     * Disable automatic slide control (manual slide control via setSlideState)
     */
    public void disableAutoSlideControl() {
        autoSlideControlEnabled = false;
    }

    /**
     * Check if automatic slide control is enabled
     * @return true if auto control is enabled
     */
    public boolean isAutoSlideControlEnabled() {
        return autoSlideControlEnabled;
    }

    // ==================== TELEMETRY HELPER METHODS ====================

    /**
     * Get the current wheel servo position (raw value 0.0-1.0)
     * @return Current position
     */
    public double getWheelPosition() {
        return intakeWheelLeft.getPosition();
    }

    /**
     * Get the current commanded wheel speed (-1.0 to 1.0)
     * @return Current speed
     */
    public double getSpeed() {
        double position = intakeWheelLeft.getPosition();
        return (position - STOP_POSITION) * 2.0;
    }

    /**
     * Get the current position of the intake slides
     * @return Current position (0.0 = in, 1.0 = out)
     */
    public double getSlidePosition() {
        return intakeSlideLeft.getPosition();
    }

    /**
     * Legacy compatibility method
     * @return Current wheel position
     */
    public double getPosition() {
        return getWheelPosition();
    }
}