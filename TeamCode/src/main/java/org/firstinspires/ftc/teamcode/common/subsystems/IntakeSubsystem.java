package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Intake subsystem for collecting and ejecting game artifacts
 * Uses a continuous rotation servo
 */
public class IntakeSubsystem {

    // Hardware
    private final Servo intakeServo; // Continuous
//    private final Servo intakeServo2; // Continuous
    private final Servo intakeSlideLeft; // Position
    private final Servo intakeSlideRight; // Position


    // Hardware configuration name - hardcoded here!
    private static final String SERVO_NAME = "intakeServo";
//    private static final String SERVO_NAME2 = "intakeServo2";
    private static final String SERVO_NAME3 = "intakeSlideLeft";
    private static final String SERVO_NAME4 = "intakeSlideRight";


    // Constants for continuous rotation
    private static final double STOP_POSITION = 0.5;

    // Constants for position
    private static final double IN_POSITION = 0.0;
    private static final double OUT_POSITION = 1.0;


    /**
     * Constructor - only needs HardwareMap
     * @param hardwareMap The hardware map from the OpMode
     */
    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(Servo.class, SERVO_NAME);
//        intakeServo2 = hardwareMap.get(Servo.class, SERVO_NAME2);
        intakeSlideLeft = hardwareMap.get(Servo.class, SERVO_NAME3);
        intakeSlideRight = hardwareMap.get(Servo.class, SERVO_NAME4);
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

    /**
     * Set the position of the intake slide
     * @param position Position from 0.0 (in) to 1.0 (out)
     */
    public void setSlidePosition(double position) { // FIXME

    }

    /**
     * Get the current position of the intake slide
     * @return Current position
     */
    public double getSlidePosition() { // FIXME
        return intakeSlideLeft.getPosition();

    }


}