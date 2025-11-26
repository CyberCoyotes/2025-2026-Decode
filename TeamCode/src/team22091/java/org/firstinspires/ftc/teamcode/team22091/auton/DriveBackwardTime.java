package org.firstinspires.ftc.teamcode.team22091.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.PinpointOdometrySubsystem;

/**
 * Basic Autonomous OpMode - Drive Backward for Time (Team 22091)
 *
 * This autonomous program drives the robot backward for a specified time period.
 * The robot will drive backward at a constant speed for 2 seconds, then stop.
 */
@Autonomous(name = "22091 Basic: Drive Backward (Time)", group = "Basic")
public class DriveBackwardTime extends LinearOpMode {

    // Subsystems
    private MecanumDriveSubsystem drive;
    private PinpointOdometrySubsystem odometry;

    // Configuration
    private static final double DRIVE_TIME = 2.0; // seconds
    private static final double DRIVE_SPEED = 0.5; // 50% power

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

        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.addLine();
        telemetry.addLine("=== AUTONOMOUS PROGRAM ===");
        telemetry.addLine("Drive Backward (Time-Based)");
        telemetry.addLine();
        telemetry.addData("Drive Time", "%.1f seconds", DRIVE_TIME);
        telemetry.addData("Drive Speed", "%.0f%%", DRIVE_SPEED * 100);
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

        // Drive backward for the specified time (negative axial)
        while (opModeIsActive() && runtime.seconds() < DRIVE_TIME) {
            // Update odometry
            odometry.update();

            // Drive backward (negative axial, no lateral, no rotation)
            drive.drive(-DRIVE_SPEED, 0, 0);

            // Display telemetry
            telemetry.addLine("=== DRIVING BACKWARD ===");
            telemetry.addData("Elapsed Time", "%.2f / %.2f sec", runtime.seconds(), DRIVE_TIME);
            telemetry.addData("Progress", "%.0f%%", (runtime.seconds() / DRIVE_TIME) * 100);
            telemetry.addLine();
            telemetry.addLine("=== POSITION ===");
            telemetry.addData("X", "%.2f in", odometry.getX(DistanceUnit.INCH));
            telemetry.addData("Y", "%.2f in", odometry.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", "%.1f°", odometry.getHeadingDegrees());
            telemetry.update();

            sleep(20); // Small delay for stability
        }

        // Stop all motors
        drive.stop();

        telemetry.addLine("=== COMPLETE ===");
        telemetry.addData("Final Position X", "%.2f in", odometry.getX(DistanceUnit.INCH));
        telemetry.addData("Final Position Y", "%.2f in", odometry.getY(DistanceUnit.INCH));
        telemetry.addData("Final Heading", "%.1f°", odometry.getHeadingDegrees());
        telemetry.update();

        // Keep telemetry visible
        sleep(2000);
    }
}
