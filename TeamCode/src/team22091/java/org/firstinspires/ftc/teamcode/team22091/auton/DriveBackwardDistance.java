package org.firstinspires.ftc.teamcode.team22091.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.PinpointOdometrySubsystem;

import static org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem.DriveState;

/**
 * Basic Autonomous OpMode - Drive Backward for Distance (Team 22091)
 *
 * This autonomous program drives the robot backward for a specified distance
 * using odometry feedback from the GoBilda Pinpoint odometry computer.
 * The robot will drive backward 6 inches, then stop.
 */
@Autonomous(name = "22091 Basic: Drive Backward (Distance)", group = "Basic")
public class DriveBackwardDistance extends LinearOpMode {

    // Subsystems
    private MecanumDriveSubsystem drive;
    private PinpointOdometrySubsystem odometry;

    // Configuration
    private static final double TARGET_DISTANCE = 6.0; // inches
    private static final double DRIVE_SPEED = 0.4; // 40% power
    private static final double DISTANCE_TOLERANCE = 0.5; // inches - stop within this range
    private static final double TIMEOUT = 5.0; // seconds - safety timeout

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
        drive.setOdometry(odometry);

        // Set to robot-centric mode for autonomous (movements relative to robot)
        drive.setState(DriveState.ROBOT_CENTRIC);

        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.addLine();
        telemetry.addLine("=== AUTONOMOUS PROGRAM ===");
        telemetry.addLine("Drive Backward (Distance-Based)");
        telemetry.addLine();
        telemetry.addData("Target Distance", "%.1f inches", TARGET_DISTANCE);
        telemetry.addData("Drive Speed", "%.0f%%", DRIVE_SPEED * 100);
        telemetry.addData("Tolerance", "±%.1f inches", DISTANCE_TOLERANCE);
        telemetry.update();

        waitForStart();

        // Reset position to (0, 0, 0) at start
        odometry.resetPosAndIMU();

        /* ========================================
         * AUTONOMOUS ROUTINE
         * ======================================== */

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        telemetry.addLine("=== DRIVING BACKWARD ===");
        telemetry.update();

        double startX = odometry.getX(DistanceUnit.INCH);

        // Drive backward until target distance is reached or timeout
        while (opModeIsActive() && runtime.seconds() < TIMEOUT) {
            // Update odometry
            odometry.update();

            // Calculate current distance traveled (X axis = forward/backward, use absolute value)
            double currentX = odometry.getX(DistanceUnit.INCH);
            double distanceTraveled = Math.abs(currentX - startX); // Use absolute value
            double distanceRemaining = TARGET_DISTANCE - distanceTraveled;

            // Check if we've reached the target
            if (Math.abs(distanceRemaining) <= DISTANCE_TOLERANCE) {
                break; // Target reached
            }

            // Drive backward (negative axial, no lateral, no rotation)
            drive.drive(-DRIVE_SPEED, 0, 0);

            // Display telemetry
            telemetry.addLine("=== DRIVING BACKWARD ===");
            telemetry.addData("Target Distance", "%.2f in", TARGET_DISTANCE);
            telemetry.addData("Distance Traveled", "%.2f in", distanceTraveled);
            telemetry.addData("Distance Remaining", "%.2f in", distanceRemaining);
            telemetry.addData("Progress", "%.0f%%", (distanceTraveled / TARGET_DISTANCE) * 100);
            telemetry.addLine();
            telemetry.addLine("=== POSITION ===");
            telemetry.addData("X (Forward)", "%.2f in", currentX);
            telemetry.addData("Y (Strafe)", "%.2f in", odometry.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", "%.1f°", odometry.getHeadingDegrees());
            telemetry.addLine();
            telemetry.addData("Elapsed Time", "%.2f sec", runtime.seconds());
            telemetry.update();

            sleep(20); // Small delay for stability
        }

        // Stop all motors
        drive.stop();

        // Calculate final distance traveled
        odometry.update();
        double finalX = odometry.getX(DistanceUnit.INCH);
        double finalDistanceTraveled = Math.abs(finalX - startX);
        double finalError = TARGET_DISTANCE - finalDistanceTraveled;
        double finalTime = runtime.seconds();

        // Freeze telemetry for debugging - stays visible until stop is pressed
        while (opModeIsActive()) {
            odometry.update();

            telemetry.addLine("=== COMPLETE ===");
            telemetry.addData("Target Distance", "%.2f in", TARGET_DISTANCE);
            telemetry.addData("Actual Distance", "%.2f in", finalDistanceTraveled);
            telemetry.addData("Error", "%.2f in", finalError);
            telemetry.addLine();
            telemetry.addLine("=== FINAL POSITION ===");
            telemetry.addData("X (Forward)", "%.2f in", finalX);
            telemetry.addData("Y (Strafe)", "%.2f in", odometry.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", "%.1f°", odometry.getHeadingDegrees());
            telemetry.addLine();
            telemetry.addData("Total Time", "%.2f sec", finalTime);
            telemetry.addData("Status", "Press STOP to end");
            telemetry.update();

            sleep(50);
        }
    }
}
