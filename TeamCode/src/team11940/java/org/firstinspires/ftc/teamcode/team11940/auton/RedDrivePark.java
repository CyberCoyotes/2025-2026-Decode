package org.firstinspires.ftc.teamcode.team11940.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.PinpointOdometrySubsystem;

/**
 * Autonomous OpMode for Red Alliance - Simple Drive and Park
 *
 * This is a template autonomous that demonstrates how to use the Pinpoint
 * odometry system for position tracking during autonomous.
 *
 * BEST PRACTICE: All autonomous programs MUST end with robot facing
 * the base wall direction (0° heading). This ensures TeleOp heading
 * reset works correctly regardless of alliance selection.
 *
 * This simple program doesn't rotate the robot, so it already complies.
 *
 * TODO: Implement actual autonomous logic for your competition strategy
 * TODO: Integrate Pedro Pathing 2.0 for trajectory following
 */
@Autonomous(name = "Red: Drive & Park", group = "Autonomous")
public class RedDrivePark extends LinearOpMode {

    // Subsystems
    private MecanumDriveSubsystem drive;
    private PinpointOdometrySubsystem odometry;

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

        // Connect odometry to drive for heading-based control
        drive.setOdometry(odometry);

        // Wait for robot to be positioned, then reset odometry to (0, 0, 0)
        telemetry.addData("Status", "Initialized - Position robot and press Start");
        telemetry.addLine();
        telemetry.addLine("IMPORTANT:");
        telemetry.addLine("- Position robot at starting location");
        telemetry.addLine("- Robot is facing forward (0°)");
        telemetry.addLine("- Odometry will reset on START");
        telemetry.update();

        waitForStart();

        // Reset position to (0, 0, 0) at start
        // Note: TeleOp will set the proper field-centric heading based on alliance
        odometry.resetPosAndIMU();

        /* ========================================
         * AUTONOMOUS ROUTINE
         * ======================================== */

        // TODO: Replace this placeholder code with actual autonomous logic

        // Example: Drive forward 24 inches
        // driveToPosition(24, 0, 0);

        // Example: Strafe right 12 inches
        // driveToPosition(24, 12, 0);

        // Example: Turn to 90 degrees
        // turnToHeading(90);

        // For now, just display telemetry
        while (opModeIsActive()) {
            odometry.update();

            telemetry.addLine("=== AUTONOMOUS ACTIVE ===");
            telemetry.addLine();
            telemetry.addLine("=== POSITION ===");
            telemetry.addData("X", "%.2f in", odometry.getX(DistanceUnit.INCH));
            telemetry.addData("Y", "%.2f in", odometry.getY(DistanceUnit.INCH));
            telemetry.addData("Heading", "%.1f°", odometry.getHeadingDegrees());
            telemetry.addLine();
            telemetry.addLine("=== STATUS ===");
            telemetry.addData("Odometry", odometry.getDeviceStatus());
            telemetry.addData("Frequency", "%.0f Hz", odometry.getPinpointFrequency());
            telemetry.addLine();
            telemetry.addLine("TODO: Implement autonomous routine");
            telemetry.addLine("TODO: Integrate Pedro Pathing 2.0");
            telemetry.update();

            sleep(50);
        }

        // Stop all motors
        drive.stop();
    }

    /* ========================================
     * AUTONOMOUS HELPER METHODS
     * ======================================== */

    /**
     * Drive to a target position using odometry feedback
     * TODO: Implement position-based driving logic
     *
     * @param targetX Target X position in inches
     * @param targetY Target Y position in inches
     * @param targetHeading Target heading in degrees
     */
    private void driveToPosition(double targetX, double targetY, double targetHeading) {
        // TODO: Implement PID control or use Pedro Pathing
        // This is a placeholder for position-based driving

        telemetry.addLine("Driving to position...");
        telemetry.addData("Target", "X: %.1f, Y: %.1f, H: %.1f", targetX, targetY, targetHeading);
        telemetry.update();
    }

    /**
     * Turn to a specific heading
     * TODO: Implement heading-based turning logic
     *
     * @param targetHeading Target heading in degrees
     */
    private void turnToHeading(double targetHeading) {
        // TODO: Implement heading control

        telemetry.addLine("Turning to heading...");
        telemetry.addData("Target Heading", "%.1f°", targetHeading);
        telemetry.update();
    }
}
