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
    
 * Robot configuration constants for Team 11940
 * These hardware names must match the names configured in the FTC Robot Controller app
 */
public class Robot11940Config {
    
    /* ========================================
     * DRIVE MOTORS (MecanumDriveSubsystem)
     * ======================================== */
    public static final String LEFT_FRONT_MOTOR = "leftFront";
    public static final String LEFT_BACK_MOTOR = "leftBack";
    public static final String RIGHT_FRONT_MOTOR = "rightFront";
    public static final String RIGHT_BACK_MOTOR = "rightBack";
    public static final String IMU = "imu";

    /* ========================================
     * INTAKE SUBSYSTEM (IntakeSubsystem)
     * ======================================== */
    public static final String INTAKE_SERVO = "intakeServo";
    public static final String INTAKE_SLIDE_LEFT = "intakeSlideLeft";
    public static final String INTAKE_SLIDE_RIGHT = "intakeSlideRight";

    /* ========================================
     * SCORING SUBSYSTEM (ScoringSubsystem)
     * ======================================== */
    public static final String TURRET_SERVO = "turretServo";
    public static final String HOOD_SERVO = "hoodServo";
    public static final String FLYWHEEL_MOTOR = "flywheelMotor";

}
