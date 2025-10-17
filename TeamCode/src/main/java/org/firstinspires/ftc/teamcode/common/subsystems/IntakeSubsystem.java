package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Intake subsystem for collecting and ejecting game artifacts
 * Uses a continuous rotation servo
 */
public class IntakeSubsystem {

    // Hardware
    private final Servo intakeServo;

    // Hardware configuration name - hardcoded here!
    private static final String SERVO_NAME = "intakeServo";

    // Constants for continuous rotation
    private static final double STOP_POSITION = 0.5;

    /**
     * Constructor - only needs HardwareMap
     * @param hardwareMap The hardware map from the OpMode
     */
    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(Servo.class, SERVO_NAME);
        stop(); // Initialize to stopped position
    }

    /**
     * Periodic method called once per scheduler run
     */
    public void periodic() {
        // Add any periodic updates here if needed
    }

    /**
     * Set the speed of the intake servo
     * @param speed Speed from -1.0 (eject) to 1.0 (intake), 0.0 is stop
     */
    public void setSpeed(double speed) {
        // Clamp speed to valid range
        speed = Math.max(-1.0, Math.min(1.0, speed));

        // Convert speed (-1.0 to 1.0) to servo position (0.0 to 1.0)
        double position = STOP_POSITION + (speed * 0.5);
        intakeServo.setPosition(position);
    }

    /**
     * Stop the intake
     */
    public void stop() {
        intakeServo.setPosition(STOP_POSITION);
    }

    /**
     * Run intake to collect artifacts at full speed
     */
    public void intakeArtifact() {
        setSpeed(1.0);
    }

    /**
     * Run intake in reverse to eject artifacts at full speed
     */
    public void ejectArtifact() {
        setSpeed(-1.0);
    }

    /**
     * Run intake to collect at a specific speed
     * @param speed Speed from 0.0 to 1.0
     */
    public void intakeArtifact(double speed) {
        setSpeed(Math.abs(speed));
    }

    /**
     * Run intake in reverse at a specific speed
     * @param speed Speed from 0.0 to 1.0
     */
    public void ejectArtifact(double speed) {
        setSpeed(-Math.abs(speed));
    }

    /**
     * Get the current servo position (raw value 0.0-1.0)
     * @return Current position
     */
    public double getPosition() {
        return intakeServo.getPosition();
    }

    /**
     * Get the current commanded speed (-1.0 to 1.0)
     * @return Current speed
     */
    public double getSpeed() {
        double position = intakeServo.getPosition();
        return (position - STOP_POSITION) * 2.0;
    }
}