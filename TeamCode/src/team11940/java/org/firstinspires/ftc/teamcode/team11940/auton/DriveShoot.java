package org.firstinspires.ftc.teamcode.team11940.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.PinpointOdometrySubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.IndexSubsystem;


import static org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem.DriveState;

/**
 * Complex Autonomous OpMode for Team 11940
 *
 * Sequence:
 * 1. Drive forward 36 inches
 * 2. Turn 45 degrees counterclockwise
 * 3. Drive forward 12 inches
 * 4. Run shooter at medium range for 5 seconds
 *
 * Uses Pinpoint odometry for accurate positioning and heading control.
 */
@Autonomous(name = "11940: Drive & Shoot", group = "Complex")
public class DriveShoot extends LinearOpMode {

    // Subsystems
    private MecanumDriveSubsystem drive;
    private PinpointOdometrySubsystem odometry;
    private ShooterSubsystem shooter;
    private IndexSubsystem index;


    // Movement configuration
    private static final double DRIVE_SPEED = 0.4; // 40% power for distance moves
    private static final double TURN_SPEED = 0.3; // 30% power for turning
    private static final double DISTANCE_TOLERANCE = 0.5; // inches
    private static final double HEADING_TOLERANCE = 2.0; // degrees
    private static final double MOVEMENT_TIMEOUT = 5.0; // seconds

    // Autonomous sequence parameters
    private static final double FIRST_DRIVE_DISTANCE = 36.0; // inches
    private static final double TURN_ANGLE = 45.0; // degrees (counterclockwise)
    private static final double SECOND_DRIVE_DISTANCE = 12.0; // inches
    private static final double SHOOTER_DURATION = 5.0; // seconds

    @Override
    public void runOpMode() throws InterruptedException {

        /* ========================================
         * INITIALIZATION
         * ======================================== */
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize subsystems
        drive = new MecanumDriveSubsystem(hardwareMap);
        odometry = new PinpointOdometrySubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        drive.setOdometry(odometry);

        // Set to robot-centric mode for autonomous
        drive.setState(DriveState.ROBOT_CENTRIC);

        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.addLine();
        telemetry.addLine("=== AUTONOMOUS SEQUENCE ===");
        telemetry.addLine("1. Drive forward 36 inches");
        telemetry.addLine("2. Turn 45° counterclockwise");
        telemetry.addLine("3. Drive forward 12 inches");
        telemetry.addLine("4. Shoot (medium) for 5 seconds");
        telemetry.update();

        waitForStart();

        // Reset position to (0, 0, 0) at start
        odometry.resetPosAndIMU();

        /* ========================================
         * AUTONOMOUS SEQUENCE
         * ======================================== */

        // Step 1: Drive forward 36 inches
        telemetry.addLine("Step 1: Driving forward 36 inches...");
        telemetry.update();
        driveDistance(FIRST_DRIVE_DISTANCE);

        sleep(500); // Brief pause between movements

        // Step 2: Turn 45 degrees counterclockwise
        telemetry.addLine("Step 2: Turning 45° counterclockwise...");
        telemetry.update();
        turnToHeading(TURN_ANGLE);

        sleep(500); // Brief pause between movements

        // Step 3: Drive forward 12 inches
        telemetry.addLine("Step 3: Driving forward 12 inches...");
        telemetry.update();
        driveDistance(SECOND_DRIVE_DISTANCE);

        sleep(500); // Brief pause before shooting

        // Step 4: Take shot at medium range for 5 seconds
        telemetry.addLine("Step 4: Taking shot...");
        telemetry.update();
        takeShot(ShooterSubsystem.ShotState.MEDIUM_RANGE, SHOOTER_DURATION);

        // Stop all systems
        drive.stop();
        shooter.stopFlywheel();

        /* ========================================
         * FINAL TELEMETRY (FROZEN)
         * ======================================== */
        displayFinalTelemetry();
    }

    /* ========================================
     * AUTONOMOUS HELPER METHODS
     * ======================================== */

    /**
     * Drive forward a specific distance using odometry feedback
     *
     * @param targetDistance Distance to drive in inches (positive = forward)
     */
    private void driveDistance(double targetDistance) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        double startX = odometry.getX(DistanceUnit.INCH);

        while (opModeIsActive() && runtime.seconds() < MOVEMENT_TIMEOUT) {
            odometry.update();

            double currentX = odometry.getX(DistanceUnit.INCH);
            double distanceTraveled = currentX - startX;
            double distanceRemaining = targetDistance - distanceTraveled;

            // Check if target reached
            if (Math.abs(distanceRemaining) <= DISTANCE_TOLERANCE) {
                break;
            }

            // Drive forward
            drive.drive(DRIVE_SPEED, 0, 0);

            // Telemetry
            telemetry.addData("Target", "%.1f in", targetDistance);
            telemetry.addData("Traveled", "%.1f in", distanceTraveled);
            telemetry.addData("Remaining", "%.1f in", distanceRemaining);
            telemetry.update();

            sleep(20);
        }

        drive.stop();
        sleep(100); // Brief stop
    }

    /**
     * Turn to a specific heading using odometry feedback
     *
     * @param targetHeading Target heading in degrees (counterclockwise positive)
     */
    private void turnToHeading(double targetHeading) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < MOVEMENT_TIMEOUT) {
            odometry.update();

            double currentHeading = odometry.getHeadingDegrees();
            double headingError = targetHeading - currentHeading;

            // Normalize error to -180 to 180
            while (headingError > 180) headingError -= 360;
            while (headingError < -180) headingError += 360;

            // Check if target reached
            if (Math.abs(headingError) <= HEADING_TOLERANCE) {
                break;
            }

            // Turn (positive error = turn left/counterclockwise)
            double turnPower = Math.signum(headingError) * TURN_SPEED;
            drive.drive(0, 0, turnPower);

            // Telemetry
            telemetry.addData("Target Heading", "%.1f°", targetHeading);
            telemetry.addData("Current Heading", "%.1f°", currentHeading);
            telemetry.addData("Error", "%.1f°", headingError);
            telemetry.update();

            sleep(20);
        }

        drive.stop();
        sleep(100); // Brief stop
    }

    /**
     * Take a shot at the specified preset for a specified duration
     * Coordinates flywheel spin-up and index motor engagement
     *
     * @param preset The shot preset (SHORT_RANGE, MEDIUM_RANGE, or LONG_RANGE)
     * @param duration Total duration in seconds
     */
    private void takeShot(ShooterSubsystem.ShotState preset, double duration) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Set the shot state (flywheel + hood)
        shooter.setShotState(preset);

        // Wait for flywheel to reach target RPM before starting index
        boolean indexStarted = false;

        while (opModeIsActive() && runtime.seconds() < duration) {
            odometry.update();

            // Once flywheel reaches target RPM, start index motors
            if (!indexStarted && shooter.isAtTargetRPM()) {
                index.runForward();  // Starts both top and bottom index motors
                indexStarted = true;
            }

            telemetry.addLine("=== SHOOTER ACTIVE ===");
            telemetry.addData("Mode", preset.name());
            telemetry.addData("Current RPM", "%.0f", shooter.getCurrentRPM());
            telemetry.addData("Target RPM", "%d", preset.getTargetRPM());
            telemetry.addData("At Target", shooter.isAtTargetRPM() ? "✓ YES" : "✗ NO");
            telemetry.addData("Index Running", indexStarted ? "YES" : "Waiting for RPM");
            telemetry.addData("Time Remaining", "%.1f sec", duration - runtime.seconds());
            telemetry.update();

            sleep(50);
        }

        // Stop both shooter and index when done
        shooter.stopFlywheel();
        index.stop();
    }

    /**
     * Display final telemetry and freeze until stop is pressed
     */
    private void displayFinalTelemetry() {
        while (opModeIsActive()) {
            odometry.update();

            telemetry.addLine("=== SEQUENCE COMPLETE ===");
            telemetry.addLine();
            telemetry.addLine("=== FINAL POSITION ===");
            telemetry.addData("X (Forward)", "%.2f in", odometry.getX(DistanceUnit.INCH));
            telemetry.addData("Y (Strafe)", "%.2f in", odometry.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", "%.1f°", odometry.getHeadingDegrees());
            telemetry.addLine();
            telemetry.addData("Status", "Press STOP to end");
            telemetry.update();

            sleep(50);
        }
    }
}
