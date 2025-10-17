package org.firstinspires.ftc.teamcode.team11940;  // ← CHANGE THIS for team22091

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.practiceBot.ShooterSubsystem;
//import org.firstinspires.ftc.teamcode.practiceBot.LimelightSubsystem;

@TeleOp(name = "MainTeleOp - 11940", group = "Competition")  // ← CHANGE NAME for team22091
public class MainTeleOp extends LinearOpMode {

    /* ========================================
     * SUBSYSTEMS
     * ======================================== */
    private IntakeSubsystem intake;
//    private ShooterSubsystem shooter;
//    private LimelightSubsystem limelight;
    // TODO: Add your drive subsystem here
    // private MecanumDriveSubsystem drive;

    /* ========================================
     * CONTROL STATE VARIABLES
     * ======================================== */
    private boolean lastDpadUpState = false;
    private boolean lastDpadDownState = false;

    // TODO: Add your button state tracking variables here
    // Example: private boolean lastAButtonState = false;

    /* ========================================
     * CONSTANTS - CUSTOMIZE THESE!
     * ======================================== */
    private static final double SLOW_MODE_MULTIPLIER = 0.6;
    private static final double SUPER_SLOW_MULTIPLIER = 0.35;
    private static final double DEADZONE = 0.1;

    // TODO: Add your team-specific constants here
    // Example: private static final double TURN_SPEED = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {

        /* ========================================
         * INITIALIZATION
         * ======================================== */
        telemetry.addData("Status", "Initializing subsystems...");
        telemetry.update();

        // Initialize all subsystems
        intake = new IntakeSubsystem(hardwareMap);
//        shooter = new ShooterSubsystem(hardwareMap);
//        limelight = new LimelightSubsystem(hardwareMap);

        // TODO: Initialize your drive subsystem
        // drive = new MecanumDriveSubsystem(hardwareMap);

        telemetry.addData("Status", "Ready to start!");
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Right Bumper - Intake");
        telemetry.addLine("Left Bumper - Eject");
//        telemetry.addLine("D-pad Up/Down - Shooter Power");
//        telemetry.addLine("X - Run Shooter");
//        telemetry.addLine("Right Trigger - Slow Mode");
//        telemetry.addLine("Left Trigger - Super Slow Mode");
        telemetry.update();

        waitForStart();

        /* ========================================
         * MAIN CONTROL LOOP
         * ======================================== */
        while (opModeIsActive()) {

            /* ========================================
             * UPDATE SUBSYSTEMS
             * ======================================== */
//            limelight.update();
            // TODO: Add periodic updates for other subsystems if needed
            // intake.periodic();

            /* ========================================
             * DRIVER 1 - DRIVE CONTROLS
             * ======================================== */
            double leftY = -gamepad1.left_stick_y;   // Forward/backward
            double leftX = gamepad1.left_stick_x;     // Strafe left/right
            double rightX = gamepad1.right_stick_x;   // Turn left/right

            // Apply deadzone
            leftY = applyDeadzone(leftY, DEADZONE);
            leftX = applyDeadzone(leftX, DEADZONE);
            rightX = applyDeadzone(rightX, DEADZONE);

            // Slow mode controls
            if (gamepad1.right_trigger > 0.6) {
                leftY *= SLOW_MODE_MULTIPLIER;
                leftX *= SLOW_MODE_MULTIPLIER;
                rightX *= SLOW_MODE_MULTIPLIER;
            }

            if (gamepad1.left_trigger > 0.35) {
                leftY *= SUPER_SLOW_MULTIPLIER;
                leftX *= SUPER_SLOW_MULTIPLIER;
                rightX *= SUPER_SLOW_MULTIPLIER;
            }

            // TODO: Send drive commands to your drive subsystem
            // drive.drive(leftY, leftX, rightX);

            /* ========================================
             * DRIVER 1 - INTAKE CONTROLS
             * ======================================== */
            if (gamepad1.right_bumper) {
                intake.intakeArtifact();
            } else if (gamepad1.left_bumper) {
                intake.ejectArtifact();
            } else {
                intake.stop();
            }

            /* ========================================
             * DRIVER 1 - SHOOTER CONTROLS
             * ======================================== */
            // D-pad UP increases shooter power

/*
            if (gamepad1.dpad_up && !lastDpadUpState) {
                shooter.increasePower();
            }
            lastDpadUpState = gamepad1.dpad_up;

            // D-pad DOWN decreases shooter power
            if (gamepad1.dpad_down && !lastDpadDownState) {
                shooter.decreasePower();
            }
            lastDpadDownState = gamepad1.dpad_down;

            // X button runs shooter
            if (gamepad1.x) {
                shooter.runShooter();
            } else {
                shooter.stopShooter();
            }
*/
            /* ========================================
             * DRIVER 2 CONTROLS (OPERATOR)
             * TODO: Add your operator controls here!
             * ======================================== */
            // Example:
            // if (gamepad2.a) {
            //     // Do something
            // }

            /* ========================================
             * TELEMETRY
             * ======================================== */
            telemetry.clearAll();

            // Drive telemetry
            telemetry.addLine("--- DRIVE ---");
            telemetry.addData("Y", "%.2f", leftY);
            telemetry.addData("X", "%.2f", leftX);
            telemetry.addData("Turn", "%.2f", rightX);
            // TODO: Add drive-specific telemetry

            // Intake telemetry
            telemetry.addLine("--- INTAKE ---");
            telemetry.addData("Speed", "%.2f", intake.getSpeed());
            telemetry.addData("Position", "%.3f", intake.getPosition());

            // Shooter telemetry
            telemetry.addLine("--- SHOOTER ---");
//            telemetry.addData("Target Power", String.format("%.0f%%", shooter.getTargetPower() * 100));
//            telemetry.addData("Motor 1", "%.2f", shooter.shooterMotor1.getPower());
//            telemetry.addData("Motor 2", "%.2f", shooter.shooterMotor2.getPower());

            // Limelight telemetry
            telemetry.addLine("--- LIMELIGHT ---");
//            telemetry.addData("Status", limelight.getStatusTelemetry());
//            telemetry.addData("AprilTags", limelight.getAprilTagTelemetry());

            // TODO: Add your custom telemetry here

            telemetry.update();

        } // End of main loop

        /* ========================================
         * CLEANUP
         * ======================================== */
        intake.stop();
//        shooter.stopShooter();
//        limelight.stop();
        // TODO: Add cleanup for your other subsystems

    } // End of runOpMode

    /* ========================================
     * HELPER METHODS
     * ======================================== */

    /**
     * Apply deadzone to joystick input
     * @param value Raw joystick value
     * @param deadzone Deadzone threshold
     * @return Processed value (0 if within deadzone)
     */
    private double applyDeadzone(double value, double deadzone) {
        return Math.abs(value) > deadzone ? value : 0.0;
    }

    // TODO: Add your helper methods here
    // Example:
    // private boolean isIntakeRunning() {
    //     return intake.getSpeed() != 0.0;
    // }

} // End of class