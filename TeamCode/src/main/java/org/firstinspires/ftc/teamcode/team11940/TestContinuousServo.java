package org.firstinspires.ftc.teamcode.team11940;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.ContinuousServoSubsystem;

@TeleOp(name = "Test: Continuous Servo")
public class TestContinuousServo extends LinearOpMode {

    private ContinuousServoSubsystem intakeServo;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize servo with the name from your robot configuration
        intakeServo = new ContinuousServoSubsystem(hardwareMap, "intakeServo");

        waitForStart();

        while (opModeIsActive()) {
            // Control with gamepad
            if (gamepad1.a) {
                intakeServo.forward();  // Full speed forward
            } else if (gamepad1.b) {
                intakeServo.reverse();  // Full speed reverse
            } else if (gamepad1.right_trigger > 0.1) {
                intakeServo.forward(gamepad1.right_trigger);  // Variable speed forward
            } else if (gamepad1.left_trigger > 0.1) {
                intakeServo.reverse(gamepad1.left_trigger);  // Variable speed reverse
            } else {
                intakeServo.stop();
            }

            // Telemetry
            telemetry.addData("Servo Position", "%.2f", intakeServo.getPosition());
            telemetry.addData("Servo Speed", "%.2f", intakeServo.getSpeed());
            telemetry.update();
        }
    }
}