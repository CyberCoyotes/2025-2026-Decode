package org.firstinspires.ftc.teamcode.common.pedropathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 2.1.2 configuration for teams 11940 and 22091.
 *
 * Both robots use identical hardware:
 *   - GoBilda Mecanum drivetrain (motors: leftFront, leftBack, rightFront, rightBack)
 *   - GoBilda Pinpoint odometry computer (I2C name: "odo")
 *   - GoBilda Swingarm dead-wheel pods
 *   - Pinpoint module: 3.75 inches RIGHT of robot center, centered front-to-back
 *
 * TUNING ORDER (run on physical robot before writing auton paths):
 *   Phase 1 — Localization (no movement required):
 *     1. ForwardLocalizer   — push robot forward, verify X increases
 *     2. StrafeLocalizer    — push robot left, verify Y increases
 *
 *   Phase 2 — Drivetrain PIDF (robot moves, start with low power):
 *     3. ForwardVelocityTuner    → sets xVelocity in MECANUM_CONSTANTS
 *     4. StrafeVelocityTuner     → sets yVelocity in MECANUM_CONSTANTS
 *     5. ForwardZeroTurn         → translational PID in FOLLOWER_CONSTANTS
 *     6. StrafeZeroTurn          → lateral PID in FOLLOWER_CONSTANTS
 *     7. TurnTest                → heading PID in FOLLOWER_CONSTANTS
 *
 *   After tuning: update xVelocity / yVelocity and the PID values below, then run auton paths.
 *
 * NOTE: Pedro Pathing manages its own GoBildaPinpointDriver instance. Do NOT instantiate
 * PinpointOdometrySubsystem and this Follower in the same OpMode — they both access "odo".
 */
public class PedroConfig {

    // ─── Pinpoint localizer ───────────────────────────────────────────────────
    // Hardware name matches PinpointOdometrySubsystem's ODOMETRY_DEVICE_NAME = "odo"
    // Swingarm pods (not 4-bar). Directions verified: X=FORWARD, Y=FORWARD for upside-down mount.
    // forwardPodY = sideways offset of the forward (X) pod = -3.75" (3.75" right of center)
    // strafePodX  = forward/back offset of the strafe (Y) pod = 0" (centered)
    public static final PinpointConstants PINPOINT_CONSTANTS = new PinpointConstants()
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardPodY(-3.75)
            .strafePodX(0.0)
            .distanceUnit(DistanceUnit.INCH)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    // ─── Mecanum drivetrain ───────────────────────────────────────────────────
    // Motor names match MecanumDriveSubsystem hardware map names ("Back" not "Rear").
    // Directions confirmed: left motors REVERSE, right motors FORWARD.
    // xVelocity / yVelocity are PLACEHOLDER values — replace after running velocity tuning.
    public static final MecanumConstants MECANUM_CONSTANTS = new MecanumConstants()
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftBack")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    // ─── Follower (path-following) constants ──────────────────────────────────
    // All PID/feedforward values are defaults and MUST be tuned on the physical robot.
    // See tuning order above.
    public static final FollowerConstants FOLLOWER_CONSTANTS = new FollowerConstants();

    // ─── Path constraints ─────────────────────────────────────────────────────
    // Conservative starting values. Increase maxVelocity (first param, 0–1 ratio of max)
    // once drivetrain is tuned and paths are verified.
    public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(0.6, 100, 1, 1);

    // ─── Factory ─────────────────────────────────────────────────────────────
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(FOLLOWER_CONSTANTS, hardwareMap)
                .pinpointLocalizer(PINPOINT_CONSTANTS)
                .mecanumDrivetrain(MECANUM_CONSTANTS)
                .pathConstraints(PATH_CONSTRAINTS)
                .build();
    }
}
