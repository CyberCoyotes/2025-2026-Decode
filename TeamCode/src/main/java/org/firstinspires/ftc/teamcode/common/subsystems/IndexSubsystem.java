package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Index subsystem for moving game artifacts through the robot
 * Uses a single DC motor to transport artifacts from intake to shooter
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
    private final DcMotor indexMotor;

    // Hardware configuration name
    private static final String INDEX_MOTOR_NAME = "indexMotor";

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
        // Initialize motor
        indexMotor = hardwareMap.get(DcMotor.class, INDEX_MOTOR_NAME);

        // Set motor to brake when power is zero for better control
        indexMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        indexMotor.setPower(power);
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
     * Get the current motor power
     * @return Current motor power (-1.0 to 1.0)
     */
    public double getMotorPower() {
        return indexMotor.getPower();
    }

    /**
     * Get the current motor position (encoder ticks)
     * @return Current position in encoder ticks
     */
    public int getMotorPosition() {
        return indexMotor.getCurrentPosition();
    }

    /**
     * Check if motor is busy (if using RUN_TO_POSITION mode)
     * @return true if motor is busy
     */
    public boolean isMotorBusy() {
        return indexMotor.isBusy();
    }
}
