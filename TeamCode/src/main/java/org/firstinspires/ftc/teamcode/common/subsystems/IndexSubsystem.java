package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Index subsystem for moving game artifacts through the robot
 * Uses two DC motors - bottom stage and top stage - to transport artifacts from intake to shooter
 */
public class IndexSubsystem {

    /**
     * State machine for index motor control
     */
    public enum IndexState {
        IDLE,      // Motor stopped
        FORWARD,   // Motor running forward
        REVERSE    // Motor running reverse
    }

    // Hardware
    private final DcMotor indexBottomMotor;
    private final DcMotor indexTopMotor;

    // Hardware configuration names
    private static final String INDEX_BOTTOM_MOTOR_NAME = "indexBottomMotor";
    private static final String INDEX_TOP_MOTOR_NAME = "indexTopMotor";

    // Motor speed constants
    private static final double FORWARD_SPEED = 1.0;   // Full speed forward
    private static final double REVERSE_SPEED = -1.0;  // Full speed reverse

    // State tracking
    private IndexState state = IndexState.IDLE;

    /**
     * Constructor - initializes hardware and sets initial state
     * @param hardwareMap The hardware map from the OpMode
     */
    public IndexSubsystem(HardwareMap hardwareMap) {
        // Initialize motors
        indexBottomMotor = hardwareMap.get(DcMotor.class, INDEX_BOTTOM_MOTOR_NAME);
        indexTopMotor = hardwareMap.get(DcMotor.class, INDEX_TOP_MOTOR_NAME);

        // Configure bottom motor for indexing
        indexBottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexBottomMotor.setDirection(DcMotor.Direction.REVERSE); // Reverse motor direction
//        indexBottomMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        indexBottomMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Configure top motor for indexing
        indexTopMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexTopMotor.setDirection(DcMotor.Direction.REVERSE); // Reverse motor direction
//        indexTopMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        indexTopMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize to safe starting state
        setState(IndexState.IDLE);
    }

    /**
     * Periodic method called once per scheduler run
     * Can be used for any periodic updates needed
     */
    public void periodic() {
        // Currently no periodic updates needed
        // This method is here for future expansion
    }

    // ==================== STATE MACHINE METHODS ====================

    /**
     * Set the index motor state and update hardware accordingly
     * @param newState The desired motor state
     */
    public void setState(IndexState newState) {
        state = newState;
        updateMotorHardware();
    }

    /**
     * Get the current motor state
     * @return Current motor state
     */
    public IndexState getState() {
        return state;
    }

    /**
     * Get state as a string for telemetry
     * @return State name
     */
    public String getStateString() {
        return state.name();
    }

    /**
     * Update motor power based on current state
     * Both motors run together in the same direction
     */
    private void updateMotorHardware() {
        double power;
        switch (state) {
            case FORWARD:
                power = FORWARD_SPEED;
                break;
            case REVERSE:
                power = REVERSE_SPEED;
                break;
            case IDLE:
            default:
                power = 0.0;
                break;
        }
        indexBottomMotor.setPower(power);
        indexTopMotor.setPower(power);
    }

    // ==================== CONVENIENCE METHODS ====================

    /**
     * Run index motor forward at full speed
     */
    public void runForward() {
        setState(IndexState.FORWARD);
    }

    /**
     * Run index motor in reverse at full speed
     */
    public void runReverse() {
        setState(IndexState.REVERSE);
    }

    /**
     * Stop the index motor
     */
    public void stop() {
        setState(IndexState.IDLE);
    }

    // ==================== TELEMETRY HELPER METHODS ====================

    /**
     * Get the current bottom motor power
     * @return Current motor power (-1.0 to 1.0)
     */
    public double getBottomMotorPower() {
        return indexBottomMotor.getPower();
    }

    /**
     * Get the current top motor power
     * @return Current motor power (-1.0 to 1.0)
     */
    public double getTopMotorPower() {
        return indexTopMotor.getPower();
    }

    /**
     * Get the current bottom motor position (encoder ticks)
     * @return Current position in encoder ticks
     */
    public int getBottomMotorPosition() {
        return indexBottomMotor.getCurrentPosition();
    }

    /**
     * Get the current top motor position (encoder ticks)
     * @return Current position in encoder ticks
     */
    public int getTopMotorPosition() {
        return indexTopMotor.getCurrentPosition();
    }

    /**
     * Check if bottom motor is busy (if using RUN_TO_POSITION mode)
     * @return true if motor is busy
     */
    public boolean isBottomMotorBusy() {
        return indexBottomMotor.isBusy();
    }

    /**
     * Check if top motor is busy (if using RUN_TO_POSITION mode)
     * @return true if motor is busy
     */
    public boolean isTopMotorBusy() {
        return indexTopMotor.isBusy();
    }

    // Legacy compatibility methods (deprecated - use specific motor methods instead)
    /**
     * @deprecated Use getBottomMotorPower() instead
     */
    @Deprecated
    public double getMotorPower() {
        return getBottomMotorPower();
    }

    /**
     * @deprecated Use getBottomMotorPosition() instead
     */
    @Deprecated
    public int getMotorPosition() {
        return getBottomMotorPosition();
    }

    /**
     * @deprecated Use isBottomMotorBusy() instead
     */
    @Deprecated
    public boolean isMotorBusy() {
        return isBottomMotorBusy();
    }
}
