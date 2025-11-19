package org.firstinspires.ftc.teamcode.team11940.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.pedropathing.follower.*;
import org.pedropathing.localization.Pose;
import org.pedropathing.pathgen.BezierCurve;
import org.pedropathing.pathgen.BezierLine;
import org.pedropathing.pathgen.Path;
import org.pedropathing.pathgen.PathChain;
import org.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.ShooterSubsystem;

/**
 * Pedro Pathing Sample Autonomous - Red Alliance
 * Team 11940
 *
 * This autonomous demonstrates:
 * - Using Pedro Pathing for trajectory following
 * - Integration with Pinpoint odometry
 * - Sample scoring sequence
 * - Parking in observation zone
 *
 * Autonomous Sequence:
 * 1. Start from starting position
 * 2. Drive to pre-load scoring position
 * 3. Score pre-loaded specimen
 * 4. Drive to sample collection area
 * 5. Collect sample and score in high basket
 * 6. Park in observation zone
 */
@Autonomous(name = "Red: Pedro Sample Auto", group = "Autonomous")
public class PedroRedSampleAuto extends LinearOpMode {

    // Subsystems
    private IntakeSubsystem intake;
    private IndexSubsystem index;
    private ShooterSubsystem shooter;

    // Pedro Pathing Follower
    private Follower follower;

    /* ========================================
     * FIELD POSITIONS (in inches)
     * Coordinate system:
     * - Origin (0, 0) at field center
     * - X: positive = right (from red alliance wall)
     * - Y: positive = forward (toward blue alliance)
     * - Heading: 0° = facing forward (toward blue alliance)
     *
     * NOTE: Red positions are mirrored from Blue across the X-axis
     * ======================================== */

    // Starting position - Red alliance, left side
    private final Pose START_POSE = new Pose(12, 63, Math.toRadians(270));

    // Specimen scoring position (at submersible)
    private final Pose SPECIMEN_SCORE_POSE = new Pose(6, 36, Math.toRadians(270));

    // Sample collection positions
    private final Pose SAMPLE_1_POSE = new Pose(48, 40, Math.toRadians(270));
    private final Pose SAMPLE_2_POSE = new Pose(58, 40, Math.toRadians(270));

    // High basket scoring position
    private final Pose HIGH_BASKET_POSE = new Pose(52, 52, Math.toRadians(225));

    // Observation zone parking position
    private final Pose PARK_POSE = new Pose(12, 12, Math.toRadians(270));

    // Paths and PathChains
    private Path scoreSpecimen;
    private PathChain collectAndScoreSample;
    private Path parkPath;

    @Override
    public void runOpMode() throws InterruptedException {

        /* ========================================
         * INITIALIZATION
         * ======================================== */
        telemetry.addData("Status", "Initializing Pedro Pathing...");
        telemetry.update();

        // Initialize Pedro Pathing Follower
        // The follower will automatically use Pinpoint odometry if configured
        follower = new Follower(hardwareMap);
        follower.setStartingPose(START_POSE);

        // Initialize subsystems
        intake = new IntakeSubsystem(hardwareMap);
        index = new IndexSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        // Build paths
        buildPaths();

        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.addLine();
        telemetry.addLine("=== AUTONOMOUS SEQUENCE ===");
        telemetry.addLine("1. Score pre-loaded specimen");
        telemetry.addLine("2. Collect and score sample");
        telemetry.addLine("3. Park in observation zone");
        telemetry.update();

        waitForStart();

        /* ========================================
         * AUTONOMOUS ROUTINE
         * ======================================== */
        if (opModeIsActive()) {

            // Step 1: Score pre-loaded specimen
            scorePreloadSpecimen();

            // Step 2: Collect sample and score in high basket
            collectAndScoreSamples();

            // Step 3: Park in observation zone
            parkInObservationZone();

            // Final telemetry
            telemetry.addData("Status", "Autonomous Complete!");
            telemetry.update();
        }
    }

    /* ========================================
     * PATH BUILDING
     * ======================================== */
    private void buildPaths() {
        // Path 1: Drive to specimen scoring position
        scoreSpecimen = new Path(new BezierLine(
                new Point(START_POSE),
                new Point(SPECIMEN_SCORE_POSE)
        ));
        scoreSpecimen.setLinearHeadingInterpolation(START_POSE.getHeading(), SPECIMEN_SCORE_POSE.getHeading());

        // Path 2: Collect sample and score in high basket (using PathChain for multiple segments)
        collectAndScoreSample = follower.pathBuilder()
                // Go to first sample
                .addPath(new BezierCurve(
                        new Point(SPECIMEN_SCORE_POSE),
                        new Point(30, 48, Point.CARTESIAN),  // Control point
                        new Point(SAMPLE_1_POSE)
                ))
                .setLinearHeadingInterpolation(SPECIMEN_SCORE_POSE.getHeading(), SAMPLE_1_POSE.getHeading())
                // Go to high basket
                .addPath(new BezierLine(
                        new Point(SAMPLE_1_POSE),
                        new Point(HIGH_BASKET_POSE)
                ))
                .setLinearHeadingInterpolation(SAMPLE_1_POSE.getHeading(), HIGH_BASKET_POSE.getHeading())
                .build();

        // Path 3: Park in observation zone
        parkPath = new Path(new BezierCurve(
                new Point(HIGH_BASKET_POSE),
                new Point(36, 36, Point.CARTESIAN),  // Control point for smooth curve
                new Point(PARK_POSE)
        ));
        parkPath.setLinearHeadingInterpolation(HIGH_BASKET_POSE.getHeading(), PARK_POSE.getHeading());
    }

    /* ========================================
     * AUTONOMOUS SEQUENCES
     * ======================================== */

    /**
     * Step 1: Score the pre-loaded specimen on the high chamber
     */
    private void scorePreloadSpecimen() {
        telemetry.addData("Step", "1 - Scoring Pre-loaded Specimen");
        telemetry.update();

        // Follow path to specimen scoring position
        follower.followPath(scoreSpecimen);

        // Wait for path to complete
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
            sleep(10);
        }

        // TODO: Add specimen scoring mechanism here
        // Example: Extend lift, release specimen, retract lift
        telemetry.addData("Action", "Scoring specimen...");
        telemetry.update();
        sleep(1000);  // Placeholder for scoring action

        telemetry.addData("Step 1", "Complete!");
        telemetry.update();
    }

    /**
     * Step 2: Collect sample and score in high basket
     */
    private void collectAndScoreSamples() {
        telemetry.addData("Step", "2 - Collecting and Scoring Sample");
        telemetry.update();

        // Follow path chain to collect sample
        follower.followPath(collectAndScoreSample);

        // Monitor path progress
        boolean sampleCollected = false;
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            // Start intake when approaching sample position
            if (!sampleCollected && isNearPose(SAMPLE_1_POSE, 5.0)) {
                telemetry.addData("Action", "Collecting sample...");
                intake.intakeArtifact();
                sleep(1500);  // Collect for 1.5 seconds
                intake.stop();
                sampleCollected = true;
            }

            updateTelemetry();
            sleep(10);
        }

        // Score in high basket
        telemetry.addData("Action", "Scoring in high basket...");
        telemetry.update();

        // Spin up shooter and score
        shooter.setFlywheelState(ShooterSubsystem.FlywheelState.MEDIUM_RANGE);
        sleep(1000);  // Wait for flywheel to reach speed

        if (shooter.isAtTargetVelocity()) {
            index.runForward();
            sleep(500);  // Feed sample
            index.stop();
        }

        shooter.stopFlywheel();

        telemetry.addData("Step 2", "Complete!");
        telemetry.update();
    }

    /**
     * Step 3: Park in observation zone
     */
    private void parkInObservationZone() {
        telemetry.addData("Step", "3 - Parking in Observation Zone");
        telemetry.update();

        // Follow path to park position
        follower.followPath(parkPath);

        // Wait for path to complete
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            updateTelemetry();
            sleep(10);
        }

        telemetry.addData("Step 3", "Complete!");
        telemetry.addData("Status", "PARKED");
        telemetry.update();
    }

    /* ========================================
     * HELPER METHODS
     * ======================================== */

    /**
     * Check if robot is near a target pose
     *
     * @param targetPose Target pose to check
     * @param threshold Distance threshold in inches
     * @return True if robot is within threshold of target
     */
    private boolean isNearPose(Pose targetPose, double threshold) {
        Pose currentPose = follower.getPose();
        double distance = Math.hypot(
                currentPose.getX() - targetPose.getX(),
                currentPose.getY() - targetPose.getY()
        );
        return distance < threshold;
    }

    /**
     * Update telemetry with current position and path following status
     */
    private void updateTelemetry() {
        Pose currentPose = follower.getPose();

        telemetry.addLine("=== PEDRO PATHING ===");
        telemetry.addData("X", "%.2f in", currentPose.getX());
        telemetry.addData("Y", "%.2f in", currentPose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("Path Status", follower.isBusy() ? "Following" : "Idle");

        telemetry.addLine();
        telemetry.addLine("=== SUBSYSTEMS ===");
        telemetry.addData("Intake", intake.getWheelStateString());
        telemetry.addData("Shooter", shooter.getFlywheelState().name());
        telemetry.addData("Shooter RPM", "%.0f / %.0f",
                (shooter.getFlywheelVelocity() / 28.0) * 60.0,
                shooter.getTargetRPM());

        telemetry.update();
    }
}
