package org.firstinspires.ftc.teamcode.team11940;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.pedropathing.follower.FollowerConstants;
import org.pedropathing.localization.Localizers;

/**
 * Pedro Pathing Configuration Constants for Team 11940
 *
 * This class contains all the configuration parameters for Pedro Pathing.
 * You'll need to tune these values to match your robot's specific characteristics.
 *
 * IMPORTANT: These values are EXAMPLES and must be tuned for your robot!
 *
 * For tuning instructions, visit:
 * https://pedropathing.com/docs/tuning/
 */
@Config
public class PedroPathingConstants {

    /* ========================================
     * ROBOT PHYSICAL PARAMETERS
     * ======================================== */

    // Distance from robot center to front/back (in inches)
    // Measure from center of robot to front edge
    public static double forwardZeroPowerAcceleration = -50.0;  // Deceleration when power = 0
    public static double lateralZeroPowerAcceleration = -50.0;

    // Mass of robot (arbitrary units, used for acceleration calculations)
    public static double mass = 1.0;

    /* ========================================
     * MOTOR CONFIGURATION
     * ======================================== */

    // Motor directions (adjust based on your wiring)
    public static DcMotorSimple.Direction leftFrontDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction leftRearDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightFrontDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightRearDirection = DcMotorSimple.Direction.FORWARD;

    /* ========================================
     * DRIVE CONSTRAINTS
     * ======================================== */

    // Maximum velocities (inches/second)
    // IMPORTANT: Measure these by running your robot at full speed!
    public static double maxForwardVelocity = 50.0;  // TODO: Measure with velocity tuner
    public static double maxLateralVelocity = 40.0;   // TODO: Measure with velocity tuner
    public static double maxAngularVelocity = Math.toRadians(180);  // radians/second

    // Maximum accelerations (inches/second^2)
    public static double maxForwardAcceleration = 40.0;
    public static double maxLateralAcceleration = 35.0;
    public static double maxAngularAcceleration = Math.toRadians(120);

    /* ========================================
     * FOLLOWER PID CONSTANTS
     * ======================================== */

    // Translational PID (controls X and Y position following)
    public static double translationalPIDkP = 0.2;  // TODO: Tune with Pedro Pathing tuner
    public static double translationalPIDkI = 0.0;
    public static double translationalPIDkD = 0.01;

    // Heading PID (controls robot rotation)
    public static double headingPIDkP = 2.0;  // TODO: Tune with Pedro Pathing tuner
    public static double headingPIDkI = 0.0;
    public static double headingPIDkD = 0.1;

    // Drive PID (controls how aggressively robot follows path)
    public static double drivePIDkP = 0.01;  // TODO: Tune with Pedro Pathing tuner
    public static double drivePIDkI = 0.0;
    public static double drivePIDkD = 0.0;

    /* ========================================
     * CENTRIPETAL FORCE CORRECTION
     * ======================================== */

    // Scaling factor for centripetal force correction when following curved paths
    // Higher = more correction. Start with 1.0 and adjust if robot cuts corners.
    public static double centripetalScaling = 0.0001;

    /* ========================================
     * PATH FOLLOWING PARAMETERS
     * ======================================== */

    // How close robot must be to end point to consider path complete (inches)
    public static double pathEndTranslationalTolerance = 0.5;
    public static double pathEndHeadingTolerance = Math.toRadians(2);  // 2 degrees

    // Timeout for path following (milliseconds)
    public static double pathEndTimeoutConstraint = 500;

    // How far ahead to look on the path (inches)
    // Larger = smoother but less precise, Smaller = more precise but can oscillate
    public static double followDistance = 12.0;

    /* ========================================
     * LOCALIZATION CONFIGURATION
     * ======================================== */

    // Localizer type - using Pinpoint (goBILDA Pinpoint Odometry Computer)
    // Pedro Pathing will automatically use the Pinpoint if it's configured in hardwareMap
    public static Localizers localizerType = Localizers.PINPOINT;

    /* ========================================
     * GETTER METHODS FOR FOLLOWER CONSTANTS
     * These methods return a FollowerConstants object with all the parameters above
     * ======================================== */

    /**
     * Get FollowerConstants configured for this robot
     * This is called by the Follower to get all configuration parameters
     */
    public static FollowerConstants getFollowerConstants() {
        FollowerConstants constants = new FollowerConstants();

        // Set all the parameters
        constants.forwardZeroPowerAcceleration = forwardZeroPowerAcceleration;
        constants.lateralZeroPowerAcceleration = lateralZeroPowerAcceleration;
        constants.mass = mass;

        constants.maxForwardVelocity = maxForwardVelocity;
        constants.maxLateralVelocity = maxLateralVelocity;
        constants.maxAngularVelocity = maxAngularVelocity;

        constants.maxForwardAcceleration = maxForwardAcceleration;
        constants.maxLateralAcceleration = maxLateralAcceleration;
        constants.maxAngularAcceleration = maxAngularAcceleration;

        constants.translationalPIDkP = translationalPIDkP;
        constants.translationalPIDkI = translationalPIDkI;
        constants.translationalPIDkD = translationalPIDkD;

        constants.headingPIDkP = headingPIDkP;
        constants.headingPIDkI = headingPIDkI;
        constants.headingPIDkD = headingPIDkD;

        constants.drivePIDkP = drivePIDkP;
        constants.drivePIDkI = drivePIDkI;
        constants.drivePIDkD = drivePIDkD;

        constants.centripetalScaling = centripetalScaling;

        constants.pathEndTranslationalTolerance = pathEndTranslationalTolerance;
        constants.pathEndHeadingTolerance = pathEndHeadingTolerance;
        constants.pathEndTimeoutConstraint = pathEndTimeoutConstraint;

        constants.followDistance = followDistance;

        return constants;
    }

    /* ========================================
     * TUNING NOTES
     * ======================================== */

    /*
     * TUNING GUIDE:
     *
     * 1. MOTOR DIRECTIONS
     *    - Run the robot forward and verify all wheels spin correctly
     *    - Adjust leftFrontDirection, etc. if needed
     *
     * 2. MAX VELOCITIES
     *    - Use Pedro Pathing's velocity tuner OpMode
     *    - Drive robot at full speed forward, lateral, and rotation
     *    - Record the maximum velocities and update above
     *
     * 3. TRANSLATIONAL PID
     *    - Start with kP = 0.2, kI = 0, kD = 0.01
     *    - Increase kP if robot is too slow to reach target
     *    - Add kD if robot oscillates around target
     *    - Only add kI if there's steady-state error
     *
     * 4. HEADING PID
     *    - Start with kP = 2.0, kI = 0, kD = 0.1
     *    - Increase kP if robot turns too slowly
     *    - Add kD if robot overshoots heading
     *
     * 5. DRIVE PID
     *    - Start with kP = 0.01
     *    - Increase if robot doesn't follow path closely enough
     *    - Decrease if robot oscillates along path
     *
     * 6. FOLLOW DISTANCE
     *    - Start with 12 inches
     *    - Decrease for tighter path following
     *    - Increase for smoother, less aggressive following
     *
     * For detailed tuning instructions:
     * https://pedropathing.com/docs/tuning/
     */
}
