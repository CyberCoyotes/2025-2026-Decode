package org.firstinspires.ftc.teamcode.practicebot;

/**
 * // RobotConfig.java - Abstract base
public abstract class RobotConfig {
    public abstract double getWheelDiameter();
    public abstract double getTrackWidth();
    // etc.
}

// PracticeRobotConfig.java
public class PracticeRobotConfig extends RobotConfig {
    public double getWheelDiameter() { return 4.0; }
    public double getTrackWidth() { return 15.5; }
}
 */

/**
 * Hardware configuration for Practice robot
 * Contains hardware device names that must match the Robot Controller configuration
 */
public class PracticeRobotConfig {
    
    // ========== SCORING SUBSYSTEM ==========
    
    /**
     * Turret servo - goBilda position-based servo for turret rotation
     * Must be configured as a Servo in the Robot Controller
     */
    public static final String TURRET_SERVO = "turretServo";
    
    /**
     * Hood servo - goBilda position-based servo for hood adjustment
     * Must be configured as a Servo in the Robot Controller
     */
    public static final String HOOD_SERVO = "hoodServo";
    
    /**
     * Flywheel motor - goBilda motor for ball launcher/shooter
     * Must be configured as a Motor (DcMotor) in the Robot Controller
     */
    public static final String FLYWHEEL_MOTOR = "flywheelMotor";
    
}
