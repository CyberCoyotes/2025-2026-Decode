package org.firstinspires.ftc.teamcode.practicebot;


/* Common Imports */
// import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;

/* Robot Specific */
import org.firstinspires.ftc.teamcode.practicebot.subsystems.LimelightSubsystem;

// @Disabled

@TeleOp(group = "TeleOp", name = "Speedy TeleOp")

public class SpeedyTeleOp extends LinearOpMode {

    private LimelightSubsystem limelightSub; // Add Limelight references
    
    @Override

    public void runOpMode() throws InterruptedException {

        /* Subsystems */
        limelightSub = new LimelightSubsystem(hardwareMap);

        // Required to initialize the subsystems when starting the OpMode
        waitForStart();

        /* Reset the motor encoder position after starting the OpMode */

        // While loop to keep the robot running
        while (opModeIsActive()) {

            // Update Limelight data
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

            // End of Button Bindings

            telemetry.clearAll(); // Clear previous telemetry data
            // Add drive telemetry
            telemetry.addLine("--- DRIVE ---");

            // Intake Subsystem
            telemetry.addLine("--- INTAKE ---");

            // Telemetry
            telemetry.addLine("--- LIMELIGHT ---");
            telemetry.addData("Status", limelightSub.getStatusTelemetry());
            telemetry.addData("AprilTags", limelightSub.getAprilTagTelemetry());

            telemetry.update();

        } // end of while loop

        // Clean up when OpMode stops
        limelightSub.stop();

    } // end of runOpMode method

}// end of the class
