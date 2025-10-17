package org.firstinspires.ftc.teamcode.team22091;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;

@TeleOp(name = "MainTeleOp 22091", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {

    /* ========================================
     * SUBSYSTEMS
     * ======================================== */
    private IntakeSubsystem intake;
    private MecanumDriveSubsystem drive;
//    private ShooterSubsystem shooter;
//    private LimelightSubsystem limelight;

    /* ========================================
     * CONTROL STATE VARIABLES
     * ======================================== */
    private boolean lastDpadUpState = false;
    private boolean lastDpadDownState = false;
    private boolean lastDpadLeftState = false;
    private boolean lastDpadRightState = false;

    /* ========================================
     * CONSTANTS - CUSTOMIZE THESE!
     * ======================================== */
    private static final double SLOW_MODE_MULTIPLIER = 0.6;
    private static final double SUPER_SLOW_MULTIPLIER = 0.35;

    // Drive configuration defaults
    private static final double DEFAULT_SPEED = 0.85;
    private static final double DEFAULT_SENSITIVITY = 1.2;
    private static final double DEFAULT_DEADZONE = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {

        /* ========================================
         * INITIALIZATION
         * ======================================== */
        telemetry.addData("Status", "Initializing subsystems...");
        telemetry.update();

        // Initialize all subsystems
        intake = new IntakeSubsystem(hardwareMap);
        drive = new MecanumDriveSubsystem(hardwareMap);
//        shooter = new ShooterSubsystem(hardwareMap);
//        limelight = new LimelightSubsystem(hardwareMap);

        // Configure drive defaults
        drive.setSpeed(DEFAULT_SPEED);
        drive.setSensitivity(DEFAULT_SENSITIVITY);
        drive.setDeadzone(DEFAULT_DEADZONE);

        telemetry.addData("Status", "Ready to start!");
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Left Stick - Drive/Strafe");
        telemetry.addLine("Right Stick X - Turn");
        telemetry.addLine("Right Trigger - Slow Mode");
        telemetry.addLine("Left Trigger - Super Slow Mode");
        telemetry.addLine("D-pad Up/Down - Speed Adjust");
        telemetry.addLine("D-pad Left/Right - Sensitivity");
        telemetry.addLine();
        telemetry.addLine("Right Bumper - Intake");
        telemetry.addLine("Left Bumper - Eject");
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

            /* ========================================
             * DRIVER 1 - DRIVE CONTROLS
             * ======================================== */
            double axial = -gamepad1.left_stick_y;    // Forward/backward
            double lateral = gamepad1.left_stick_x;    // Strafe left/right
            double yaw = gamepad1.right_stick_x;       // Turn left/right

            // Temporary speed modifiers (don't affect base speed setting)
            double tempSpeedMultiplier = 1.0;

            if (gamepad1.right_trigger > 0.6) {
                tempSpeedMultiplier = SLOW_MODE_MULTIPLIER;
            }
            if (gamepad1.left_trigger > 0.35) {
                tempSpeedMultiplier = SUPER_SLOW_MULTIPLIER;
            }

            // Apply temporary speed multiplier
            double currentSpeed = drive.getSpeed();
            drive.setSpeed(currentSpeed * tempSpeedMultiplier);

            // Send drive command
            drive.drive(axial, lateral, yaw);

            // Restore original speed
            drive.setSpeed(currentSpeed);

            /* ========================================
             * DRIVER 1 - DRIVE CONFIGURATION CONTROLS
             * ======================================== */
            // D-pad UP increases base speed
            if (gamepad1.dpad_up && !lastDpadUpState) {
                drive.increaseSpeed();
            }
            lastDpadUpState = gamepad1.dpad_up;

            // D-pad DOWN decreases base speed
            if (gamepad1.dpad_down && !lastDpadDownState) {
                drive.decreaseSpeed();
            }
            lastDpadDownState = gamepad1.dpad_down;

            // D-pad RIGHT increases sensitivity
            if (gamepad1.dpad_right && !lastDpadRightState) {
                drive.increaseSensitivity();
            }
            lastDpadRightState = gamepad1.dpad_right;

            // D-pad LEFT decreases sensitivity
            if (gamepad1.dpad_left && !lastDpadLeftState) {
                drive.decreaseSensitivity();
            }
            lastDpadLeftState = gamepad1.dpad_left;

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
/*
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
            telemetry.addLine("=== DRIVE ===");
            telemetry.addData("Speed", String.format("%.0f%%", drive.getSpeed() * 100));
            telemetry.addData("Sensitivity", String.format("%.1fx", drive.getSensitivity()));
            telemetry.addData("Deadzone", String.format("%.2f", drive.getDeadzone()));
            telemetry.addLine();
            telemetry.addData("Left Front", "%.2f", drive.getLeftFrontPower());
            telemetry.addData("Right Front", "%.2f", drive.getRightFrontPower());
            telemetry.addData("Left Back", "%.2f", drive.getLeftBackPower());
            telemetry.addData("Right Back", "%.2f", drive.getRightBackPower());

            // Intake telemetry
            telemetry.addLine();
            telemetry.addLine("=== INTAKE ===");
            telemetry.addData("Speed", "%.2f", intake.getSpeed());
            telemetry.addData("Position", "%.3f", intake.getPosition());

            // Shooter telemetry
//            telemetry.addLine();
//            telemetry.addLine("=== SHOOTER ===");
//            telemetry.addData("Target Power", String.format("%.0f%%", shooter.getTargetPower() * 100));
//            telemetry.addData("Motor 1", "%.2f", shooter.shooterMotor1.getPower());
//            telemetry.addData("Motor 2", "%.2f", shooter.shooterMotor2.getPower());

            // Limelight telemetry
//            telemetry.addLine();
//            telemetry.addLine("=== LIMELIGHT ===");
//            telemetry.addData("Status", limelight.getStatusTelemetry());
//            telemetry.addData("AprilTags", limelight.getAprilTagTelemetry());

            telemetry.update();

        } // End of main loop

        /* ========================================
         * CLEANUP
         * ======================================== */
        drive.stop();
        intake.stop();
//        shooter.stopShooter();
//        limelight.stop();

    } // End of runOpMode

} // End of class