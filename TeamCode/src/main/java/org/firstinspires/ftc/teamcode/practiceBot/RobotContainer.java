package org.firstinspires.ftc.teamcode.practiceBot;

// import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// @Disabled

@TeleOp(group = "drive", name = "TeleOp")

public class RobotContainer extends LinearOpMode {

    private ShooterSubsystem shooterSub;
    private boolean lastDpadUpState = false;
    private boolean lastDpadDownState = false;
    private LimelightSubsystem limelightSub; // Add Limelight references
    limelightSub = new LimelightSubsystem(hardwareMap); // Initialize Limelight


    @Override

    public void runOpMode() throws InterruptedException {

        /* Subsystems */
        shooterSub = new ShooterSubsystem(hardwareMap);

        // Required to initialize the subsystems when starting the OpMode
        waitForStart();

        /* Reset the motor encoder position after starting the OpMode */

        // While loop to keep the robot running
        while (opModeIsActive()) {
            // Main loop
            limelightSub.update();

            /* Driver 1 Controls (Movement & Intake Arm) */
            // Driving controls
            double leftY = -gamepad1.left_stick_y; // Reversed to match forward direction
            double leftX = gamepad1.left_stick_x;
            double rightX = gamepad1.right_stick_x;

            // Apply deadzone to prevent drift
            leftY = Math.abs(leftY) > 0.1 ? leftY : 0;
            leftX = Math.abs(leftX) > 0.1 ? leftX : 0;
            rightX = Math.abs(rightX) > 0.1 ? rightX : 0;

            // Optional: Add fine control mode

            // Slow-mo
            if (gamepad1.right_trigger > 0.6) {
                leftY *= 0.6;
                leftX *= 0.6;
                rightX *= 0.6;
            }

            // Super slow-mo
            if (gamepad1.left_trigger > 0.35) {
                leftY *= 0.35;
                leftX *= 0.35;
                rightX *= 0.35;
            }

            // Update drive with new powers

            // Shooter controls
            // D-pad UP increases power by 10%
            if (gamepad1.dpad_up && !lastDpadUpState) {
                shooterSub.increasePower();
            }
            lastDpadUpState = gamepad1.dpad_up;

            // D-pad DOWN decreases power by 10%
            if (gamepad1.dpad_down && !lastDpadDownState) {
                shooterSub.decreasePower();
            }
            lastDpadDownState = gamepad1.dpad_down;

            // X button runs the shooter at target power, release stops it
            // Note: Using X instead of A to avoid conflict with intake arm control
            if (gamepad1.x) {
                shooterSub.runShooter();
            } else {
                shooterSub.stopShooter();
            }

            // End of Button Bindings

            telemetry.clearAll(); // Clear previous telemetry data
            // Add drive telemetry
            telemetry.addLine("--- DRIVE ---");

            // Intake Subsystem
            telemetry.addLine("--- INTAKE ---");

            // Shooter Subsystem
            telemetry.addLine("--- SHOOTER ---");
            telemetry.addData("Shooter Target Power",String.format("%.0f%%",shooterSub.getTargetPower() * 100));
            telemetry.addData("Motor 1 Power",String.format("%.2f",shooterSub.shooterMotor1.getPower()));
            telemetry.addData("Motor 2 Power",String.format("%.2f",shooterSub.shooterMotor2.getPower()));

            // Telemetry
            telemetry.addLine("--- LIMELIGHT ---");
            telemetry.addData("Status", limelightSub.getStatusTelemetry());
            telemetry.addData("AprilTags", limelightSub.getAprilTagTelemetry());

            telemetry.update();

        } // end of while loop

        // Cleanup
        // limelightSub.stop();

    } // end of runOpMode method

}// end of the class
