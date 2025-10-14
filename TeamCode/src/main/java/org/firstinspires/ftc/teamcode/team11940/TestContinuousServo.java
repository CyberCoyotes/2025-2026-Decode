package org.firstinspires.ftc.teamcode.team11940;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

@TeleOp(name = "Test: Intake Servo", group = "test")
public class TestContinuousServo extends LinearOpMode {

    private IntakeSubsystem intake;  // More FRC-like variable name

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize intake subsystem - no name needed!
        intake = new IntakeSubsystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  Right Bumper - Intake");
        telemetry.addLine("  Left Bumper  - Eject");
        telemetry.addLine("  Right Trigger - Variable Intake");
        telemetry.addLine("  Left Trigger  - Variable Eject");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Control with gamepad
            if (gamepad1.right_bumper) {
                intake.intakeArtifact();  // Full speed intake
            } else if (gamepad1.left_bumper) {
                intake.ejectArtifact();   // Full speed eject
            } else if (gamepad1.right_trigger > 0.1) {
                intake.intakeArtifact(gamepad1.right_trigger);  // Variable speed intake
            } else if (gamepad1.left_trigger > 0.1) {
                intake.ejectArtifact(gamepad1.left_trigger);    // Variable speed eject (fixed!)
            } else {
                intake.stop();
            }

            // Telemetry
            telemetry.addLine("--- INTAKE SUBSYSTEM ---");
            telemetry.addData("Position", "%.3f", intake.getPosition());
            telemetry.addData("Speed", "%.2f", intake.getSpeed());

            // Show state
            String state = "STOPPED";
            if (gamepad1.right_bumper || gamepad1.right_trigger > 0.1) {
                state = "INTAKE";
            } else if (gamepad1.left_bumper || gamepad1.left_trigger > 0.1) {
                state = "EJECT";
            }
            telemetry.addData("State", state);

            telemetry.update();
        }
    }
}