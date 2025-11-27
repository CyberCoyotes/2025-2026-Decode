package org.firstinspires.ftc.teamcode.team11940.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.PinpointOdometrySubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.ShooterSubsystem;

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

    /* ========================================
     * ALLIANCE SELECTION
     * ======================================== */
    public enum Alliance {
        BLUE(90.0),      // Blue alliance: 90° heading offset (base wall is 90° CCW)
        RED(270.0);      // Red alliance: 270° heading offset (180° from blue)

        private final double headingOffset;

        Alliance(double headingOffset) {
            this.headingOffset = headingOffset;
        }

        public double getHeadingOffset() {
            return headingOffset;
        }
    }

    // Subsystems
    private MecanumDriveSubsystem drive;
    private PinpointOdometrySubsystem odometry;
    private ShooterSubsystem shooter;

    // Alliance configuration
    private Alliance selectedAlliance = Alliance.BLUE;  // Default to BLUE

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

        // Alliance selection loop
        boolean lastXButton = false;
        boolean lastBButton = false;

        while (!isStarted() && !isStopRequested()) {
            // Toggle alliance selection with X (BLUE) and B (RED) buttons
            boolean currentXButton = gamepad1.x;
            boolean currentBButton = gamepad1.b;

            if (currentXButton && !lastXButton) {
                selectedAlliance = Alliance.BLUE;
            } else if (currentBButton && !lastBButton) {
                selectedAlliance = Alliance.RED;
            }

            lastXButton = currentXButton;
            lastBButton = currentBButton;

            telemetry.addData("Status", "Initialized - Ready to Start");
            telemetry.addLine();
            telemetry.addData(">>> SELECTED ALLIANCE <<<", selectedAlliance.name());
            telemetry.addData("Heading Offset", "%.0f°", selectedAlliance.getHeadingOffset());
            telemetry.addLine();
            telemetry.addLine("Press X for BLUE alliance");
            telemetry.addLine("Press B for RED alliance");
            telemetry.addLine();
            telemetry.addLine("=== AUTONOMOUS SEQUENCE ===");
            telemetry.addLine("1. Drive forward 36 inches");
            telemetry.addLine("2. Turn 45° counterclockwise");
            telemetry.addLine("3. Drive forward 12 inches");
            telemetry.addLine("4. Shoot (medium) for 5 seconds");
            telemetry.update();
        }

        // Reset position and set alliance-specific heading
        // This heading will be preserved when transitioning to TeleOp
        odometry.resetPosAndIMU(selectedAlliance.getHeadingOffset());

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

        // Step 4: Run shooter for 5 seconds
        telemetry.addLine("Step 4: Activating shooter...");
        telemetry.update();
        runShooter(SHOOTER_DURATION);

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
     * Run the shooter at medium range for a specified duration
     *
     * @param duration Duration in seconds
     */
    private void runShooter(double duration) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        // Activate shooter at medium range
        shooter.setFlywheelState(ShooterSubsystem.FlywheelState.MEDIUM_RANGE);

        while (opModeIsActive() && runtime.seconds() < duration) {
            odometry.update();

            telemetry.addLine("=== SHOOTER ACTIVE ===");
            telemetry.addData("Mode", "MEDIUM RANGE");
            telemetry.addData("Current RPM", "%.0f", shooter.getCurrentRPM());
            telemetry.addData("Target RPM", "%d", ShooterSubsystem.FlywheelState.MEDIUM_RANGE.getTargetRPM());
            telemetry.addData("Time Remaining", "%.1f sec", duration - runtime.seconds());
            telemetry.update();

            sleep(50);
        }

        shooter.stopFlywheel();
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
