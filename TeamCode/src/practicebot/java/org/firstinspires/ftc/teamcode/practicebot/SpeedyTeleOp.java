package org.firstinspires.ftc.teamcode.practicebot;

/* Common Imports */
// import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.practicebot.subsystems.SpeedyDriveSubsystem;

/* Robot Specific */
import org.firstinspires.ftc.teamcode.practicebot.subsystems.LimelightSubsystem;

// @Disabled

@TeleOp(group = "TeleOp", name = "Speedy TeleOp")

public class SpeedyTeleOp extends LinearOpMode {

    /* ========================================
     * SUBSYSTEMS
     * ======================================== */
    private SpeedyDriveSubsystem drive;
//    private LimelightSubsystem limelightSub;

    /* ========================================
     * CONTROL STATE VARIABLES
     * ======================================== */
    private boolean lastDpadUpState = false;
    private boolean lastDpadDownState = false;
    private boolean lastDpadLeftState = false;
    private boolean lastDpadRightState = false;
    private boolean lastOptionsState = false;      // For field-centric toggle
    private boolean lastShareState = false;        // For heading reset

    /* ========================================
     * CONSTANTS - CUSTOMIZE THESE!
     * ======================================== */
    private static final double SLOW_MODE_MULTIPLIER = 0.6;
    private static final double SUPER_SLOW_MULTIPLIER = 0.35;

    // Drive configuration defaults
    private static final double DEFAULT_SPEED = 0.50;
    private static final double DEFAULT_SENSITIVITY = 1.2;
    private static final double DEFAULT_DEADZONE = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {

        /* ========================================
         * INITIALIZATION
         * ======================================== */
        telemetry.addData("Status", "Initializing subsystems...");
        telemetry.update();

        // Initialize subsystems
        drive = new SpeedyDriveSubsystem(hardwareMap);
//        limelightSub = new LimelightSubsystem(hardwareMap);

        // Configure drive defaults
        drive.setSpeed(DEFAULT_SPEED);
        drive.setSensitivity(DEFAULT_SENSITIVITY);
        drive.setDeadzone(DEFAULT_DEADZONE);
        drive.setFieldCentric(true);  // Start in field-centric mode

        telemetry.addData("Status", "Ready to start!");
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("Left Stick - Drive/Strafe");
        telemetry.addLine("Right Stick X - Turn");
        telemetry.addLine("Right Trigger - Slow Mode");
        telemetry.addLine("Left Trigger - Super Slow Mode");
        telemetry.addLine();
        telemetry.addLine("D-pad Up/Down - Speed Adjust");
        telemetry.addLine("D-pad Left/Right - Sensitivity");
        telemetry.addLine();
        telemetry.addLine("Options - Toggle Field/Robot Mode");
        telemetry.addLine("Share - Reset Heading (0°)");
        telemetry.addLine();
        telemetry.addData("Drive Mode", "FIELD-CENTRIC");
        telemetry.update();

        // Required to initialize the subsystems when starting the OpMode
        waitForStart();

        // Reset heading at start - current direction is "forward"
        drive.resetHeading();

        /* ========================================
         * MAIN CONTROL LOOP
         * ======================================== */
        while (opModeIsActive()) {

            /* ========================================
             * UPDATE SUBSYSTEMS
             * ======================================== */
//            limelightSub.update();

            /* ========================================
             * DRIVER 1 - DRIVE CONTROLS
             * ======================================== */
            double axial = -gamepad1.left_stick_y;    // Forward/backward
            double lateral = -gamepad1.left_stick_x;    // Strafe left/right //TODO Test this fix
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

            // Send drive command (subsystem handles field-centric transformation)
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

            // A Button - Run left front motor while held
            /*
            if (gamepad1.a) {
                drive.setLeftFrontPower(0.5);  // Run at 50% power
                telemetry.addData("LF Motor", "RUNNING (50%)");
            } else {
                drive.setLeftFrontPower(0.0);  // Stop when released
            }

             */




            /* ========================================
             * DRIVER 1 - FIELD-CENTRIC CONTROLS
             * ======================================== */
            // OPTIONS button (☰) - Toggle between field-centric and robot-centric
            if (gamepad1.options && !lastOptionsState) {
                boolean newMode = drive.toggleFieldCentric();
                // Optional: Add a rumble or sound effect here to confirm toggle
                gamepad1.rumble(200);  // Short rumble feedback
            }
            lastOptionsState = gamepad1.options;

            // SHARE button (⧉) - Reset heading to 0° (redefine "forward")
            if (gamepad1.share && !lastShareState) {
                drive.resetHeading();
                // Optional: Add feedback
                gamepad1.rumble(500);  // Longer rumble to distinguish from mode toggle
            }
            lastShareState = gamepad1.share;

            /* ========================================
             * TELEMETRY
             * ======================================== */
            telemetry.clearAll();

            // Drive telemetry
            telemetry.addLine("=== DRIVE ===");
            telemetry.addData("Mode", drive.isFieldCentric() ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            telemetry.addData("Heading", "%.1f°", drive.getHeading());
            telemetry.addData("Speed", String.format("%.0f%%", drive.getSpeed() * 100));
            telemetry.addData("Sensitivity", String.format("%.1fx", drive.getSensitivity()));
            telemetry.addData("Deadzone", String.format("%.2f", drive.getDeadzone()));
            telemetry.addLine();
            telemetry.addData("Left Front", "%.2f", drive.getLeftFrontPower());
            telemetry.addData("Right Front", "%.2f", drive.getRightFrontPower());
            telemetry.addData("Left Back", "%.2f", drive.getLeftBackPower());
            telemetry.addData("Right Back", "%.2f", drive.getRightBackPower());

            // Limelight telemetry
            telemetry.addLine();
            telemetry.addLine("=== LIMELIGHT ===");
//            telemetry.addData("Status", limelightSub.getStatusTelemetry());
//            telemetry.addData("AprilTags", limelightSub.getAprilTagTelemetry());

            // Control hints
            telemetry.addLine();
            telemetry.addLine("=== QUICK CONTROLS ===");
            telemetry.addLine("Options: Toggle Drive Mode");
            telemetry.addLine("Share: Reset Heading");

            telemetry.update();

        } // end of while loop

        /* ========================================
         * CLEANUP
         * ======================================== */
        drive.stop();

//        limelightSub.stop();

    } // end of runOpMode method

} // end of the class