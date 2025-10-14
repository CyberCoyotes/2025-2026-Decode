package org.firstinspires.ftc.teamcode.team11940;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem for controlling a continuous rotation servo (like GoBilda servos)
 * In continuous mode:
 * - 0.0 = full speed reverse
 * - 0.5 = stop
 * - 1.0 = full speed forward
 */
public class ContinuousServoSubsystem {

    // Hardware map and servo object
    private final Servo servo;

    // Constants for continuous rotation
    private static final double STOP_POSITION = 0.5;
    private static final double FORWARD_RANGE = 0.5;  // 0.5 to 1.0
    private static final double REVERSE_RANGE = 0.5;  // 0.0 to 0.5

    /**
     * Constructor
     * @param hardwareMap The hardware map from the OpMode
     * @param name The name of the servo in the robot configuration
     */
    public ContinuousServoSubsystem(HardwareMap hardwareMap, String name) {
        servo = hardwareMap.get(Servo.class, name);
        stop(); // Initialize to stopped position
    }

    /**
     * Periodic method called once per scheduler run
     * Can be used for telemetry or state management
     */
    public void periodic() {
        // Add any periodic updates here if needed
    }

    /**
     * Set the speed of the continuous rotation servo
     * @param speed Speed from -1.0 (full reverse) to 1.0 (full forward), 0.0 is stop
     */
    public void setSpeed(double speed) {
        // Clamp speed to valid range
        speed = Math.max(-1.0, Math.min(1.0, speed));

        // Convert speed (-1.0 to 1.0) to servo position (0.0 to 1.0)
        double position = STOP_POSITION + (speed * 0.5);
        servo.setPosition(position);
    }

    /**
     * Stop the servo
     */
    public void stop() {
        servo.setPosition(STOP_POSITION);
    }

    /**
     * Run forward at full speed
     */
    public void forward() {
        setSpeed(1.0);
    }

    /**
     * Run reverse at full speed
     */
    public void reverse() {
        setSpeed(-1.0);
    }

    /**
     * Run forward at a specific speed
     * @param speed Speed from 0.0 to 1.0
     */
    public void forward(double speed) {
        setSpeed(Math.abs(speed));
    }

    /**
     * Run reverse at a specific speed
     * @param speed Speed from 0.0 to 1.0
     */
    public void reverse(double speed) {
        setSpeed(-Math.abs(speed));
    }

    /**
     * Get the current servo position (raw value 0.0-1.0)
     * Note: This returns the commanded position, not actual position/speed
     * @return Current position
     */
    public double getPosition() {
        return servo.getPosition();
    }

    /**
     * Get the current commanded speed (-1.0 to 1.0)
     * @return Current speed
     */
    public double getSpeed() {
        double position = servo.getPosition();
        return (position - STOP_POSITION) * 2.0;
    }
}