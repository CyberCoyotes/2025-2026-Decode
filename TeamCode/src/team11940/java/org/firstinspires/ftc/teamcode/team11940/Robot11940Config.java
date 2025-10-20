package org.firstinspires.ftc.teamcode.team11940;

/**
 * Hardware configuration for Team 11940 robot
 * Contains hardware device names that must match the Robot Controller configuration
 */
public class Robot11940Config {
    
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
