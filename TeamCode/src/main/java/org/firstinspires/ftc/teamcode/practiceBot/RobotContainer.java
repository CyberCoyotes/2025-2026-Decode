package org.firstinspires.ftc.teamcode.practiceBot;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BucketSubsystem;

// @Disabled

@TeleOp(group = "drive", name = "TeleOp")

public class RobotContainer extends LinearOpMode {


    private MecanumDrive drive;
    private ShooterSubsystem shooterSub;
    private boolean lastDpadUpState = false;
    private boolean lastDpadDownState = false;

    @Override

    public void runOpMode() throws InterruptedException {

        /* Subsystems */
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        sensors = new Sensors(hardwareMap);
        bucketSub = new BucketSubsystem(hardwareMap,sensors);
        shooterSub = new ShooterSubsystem(hardwareMap);

        bucketSub.setIntakeSubsystem(intakeSub);
        intakeSub.setBucketSubsystem(bucketSub);

        // Required to initialize the subsystems when starting the OpMode
        waitForStart();

        /* Reset the motor encoder position after starting the OpMode */

        // While loop to keep the robot running
        while (opModeIsActive()) {
            // Redundant?
            bucketSub.setIntakeSubsystem(intakeSub);
            intakeSub.setBucketSubsystem(bucketSub);

            /* Touch Sensors */
            slidesSub.resetSlideEncoderOnTouch();
            bucketSub.resetLiftEncoderOnTouch();

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
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            leftY,  // Forward/backward
                            -leftX    // Left/right - negative for correct strafing direction
                    ),
                    -rightX     // Turning - negative for correct turning direction
            ));

            drive.updatePoseEstimate();

            // Intake arm controls
            if (gamepad1.a) intakeSub.setIntakeArm(ARM_POSE_DOWN);
            if (gamepad1.b) intakeSub.setIntakeArm(ARM_POSE_MID);
            if (gamepad1.y) intakeSub.setIntakeArm(ARM_POSE_UP);

            // Intake wheel controls
            // if (gamepad1.right_bumper) intakeSub.powerIntakeWheel(WHEEL_INTAKE);
            if (gamepad1.right_bumper) intakeSub.smartPowerIntakeWheel(1.0);
            else if (gamepad1.left_bumper) intakeSub.smartPowerIntakeWheel(-1.0);
            else intakeSub.powerIntakeWheel(0);

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

            /* Driver 2 Controls (Scoring & Mechanisms) */
            // Bucket controls
            if (gamepad2.a) bucketSub.setBucket(BUCKET_DOWN);
            if (gamepad2.b) bucketSub.setBucket(BUCKET_MID);
            if (gamepad2.y) bucketSub.setBucket(BUCKET_UP);

            // Lift controls
            if (gamepad2.dpad_up) bucketSub.moveLiftUp();
            else if (gamepad2.dpad_left) bucketSub.setLiftLow();
            else if (gamepad2.dpad_right) bucketSub.setLiftHigh();
            else if (gamepad2.dpad_down) bucketSub.setLiftDown();

            // Slides control with Triggers
            slidesSub.controlSlides(gamepad2);

            // Utility controls
            if (gamepad2.back) bucketSub.tareLift();

            // End of Button Bindings

            telemetry.clearAll(); // Clear previous telemetry data
            // Add drive telemetry
            telemetry.addLine("--- DRIVE ---");
            telemetry.addData("X Position", String.format("%.2f", drive.pose.position.x));
            telemetry.addData("Y Position", String.format("%.2f", drive.pose.position.y));
            telemetry.addData("Heading", String.format("%.2f°", Math.toDegrees(drive.pose.heading.toDouble())));

            // Intake Subsystem
            telemetry.addLine("--- INTAKE ---");
            telemetry.addData("Wheel Power",String.format("%.2f",intakeSub.intakeWheel.getPower()));
            telemetry.addData("",intakeSub.getIntakeArmStatus().getDescription());
            telemetry.addData("Arm",String.format("%.2f",intakeSub.intakeArm.getPosition()));
            telemetry.addData("Sample",sensors.getSampleStatus().getDescription());

            // Slide Subsystem
            telemetry.addLine("--- SLIDE ---");
            telemetry.addData("Slide",String.format("%s, (%d)",slidesSub.getSlideStatus(),slidesSub.slide.getCurrentPosition()));
            telemetry.addData("Touch Sensor",sensors.isSlideTouchPressed());

            // Bucket Subsystem
            telemetry.addLine("--- BUCKET ---");
            telemetry.addData("Bucket Status",String.format("%s, (%.2f)",bucketSub.getBucketStatus(),bucketSub.bucketServo.getPosition()));
            telemetry.addData("Lift Status",String.format("%s, (%d)",bucketSub.getLiftStatus(),bucketSub.lift.getCurrentPosition()));
            telemetry.addData("Lift Motor Power",String.format("%.2f A",bucketSub.lift.getPower()));

            // Shooter Subsystem
            telemetry.addLine("--- SHOOTER ---");
            telemetry.addData("Shooter Target Power",String.format("%.0f%%",shooterSub.getTargetPower() * 100));
            telemetry.addData("Motor 1 Power",String.format("%.2f",shooterSub.shooterMotor1.getPower()));
            telemetry.addData("Motor 2 Power",String.format("%.2f",shooterSub.shooterMotor2.getPower()));

            bucketSub.updateLift();
            telemetry.update();

        } // end of while loop

    } // end of runOpMode method

}// end of the class
