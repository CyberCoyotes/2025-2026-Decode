package org.firstinspires.ftc.teamcode.team22091;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.PinpointOdometrySubsystem;

@TeleOp(name = "Vanilla TeleOp", group = "TeleOp")
@Disabled

public class VanillaTeleOp extends LinearOpMode {

    /* ========================================
     * ALLIANCE SELECTION
     * ======================================== */
    public enum Alliance {
        BLUE(90.0),      // Blue alliance: 90° heading offset (base wall is 90° CCW)
        RED(270.0);      // Red alliance: 270° heading offset (180° from blue)

        private final double headingOffset;

        Alliance(double headingOffset) {
            this.headingOffset = headingOffset;
        }

        public double getHeadingOffset() {
            return headingOffset;
        }
    }

    /* ========================================
     * SUBSYSTEMS
     * ======================================== */
//    private IntakeSubsystem intake;
    private MecanumDriveSubsystem drive;
    private PinpointOdometrySubsystem odometry;
//    private ShooterSubsystem shooter;

    /* ========================================
     * ALLIANCE CONFIGURATION
     * ======================================== */
    private Alliance selectedAlliance = Alliance.BLUE;  // Default to BLUE

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
//        intake = new IntakeSubsystem(hardwareMap);
        drive = new MecanumDriveSubsystem(hardwareMap);
        odometry = new PinpointOdometrySubsystem(hardwareMap);
//        shooter = new ShooterSubsystem(hardwareMap);

        // Connect odometry to drive subsystem for heading information
        drive.setOdometry(odometry);

        // Configure drive defaults
        drive.setSpeed(DEFAULT_SPEED);
        drive.setSensitivity(DEFAULT_SENSITIVITY);
        drive.setDeadzone(DEFAULT_DEADZONE);
        drive.setFieldCentric(true);  // Start in field-centric mode

        // Alliance selection loop
        boolean lastXButton = false;
        boolean lastBButton = false;

        while (!isStarted() && !isStopRequested()) {
            // Toggle alliance selection with X (BLUE) and B (RED) buttons
            boolean currentXButton = gamepad1.x;
            boolean currentBButton = gamepad1.b;

            if (currentXButton && !lastXButton) {
                selectedAlliance = Alliance.BLUE;
            } else if (currentBButton && !lastBButton) {
                selectedAlliance = Alliance.RED;
            }

            lastXButton = currentXButton;
            lastBButton = currentBButton;

            telemetry.addData("Status", "Waiting for START");
            telemetry.addLine();
            telemetry.addLine("**********************************************");
            telemetry.addLine("*** NOTICE: SELECT YOUR ALLIANCE COLOR! ***");
            telemetry.addLine("**********************************************");
            telemetry.addLine();
            telemetry.addData(">>> SELECTED ALLIANCE <<<", selectedAlliance.name());
            telemetry.addData("Heading Offset", "%.0f°", selectedAlliance.getHeadingOffset());
            telemetry.addLine();
            telemetry.addLine(">>> Press X (BLUE) for BLUE Alliance <<<");
            telemetry.addLine(">>> Press B (RED) for RED Alliance <<<");
            telemetry.addLine();
            telemetry.addLine("NOTE: This setting is only used if you");
            telemetry.addLine("      run TeleOp WITHOUT autonomous first.");
            telemetry.addLine("      Heading from autonomous is preserved.");
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
            telemetry.addLine("Share - Reset Heading");
            telemetry.addLine();
            telemetry.addData("Drive Mode", "FIELD-CENTRIC");
            telemetry.update();
        }

        // HEADING MANAGEMENT:
        // If you ran autonomous first, heading is already set correctly - DON'T reset it!
        // The line below is commented out to preserve heading from autonomous.
        //
        // ONLY uncomment this if you're running TeleOp standalone (without autonomous)
        // for testing purposes:
        // drive.resetHeading(selectedAlliance.getHeadingOffset());

        /* ========================================
         * MAIN CONTROL LOOP
         * ======================================== */
        while (opModeIsActive()) {

            // Update odometry
            odometry.update();

            /* ========================================
             * DRIVER 1 - DRIVE CONTROLS
             * ======================================== */
            double axial = -gamepad1.left_stick_y;     // Forward/backward
            double lateral = -gamepad1.left_stick_x;   // Strafe left/right (inverted)
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

            /* ========================================
             * DRIVER 1 - FIELD-CENTRIC CONTROLS
             * ======================================== */
            // OPTIONS button (☰) - Toggle between field-centric and robot-centric
            if (gamepad1.options && !lastOptionsState) {
                boolean newMode = drive.toggleFieldCentric();
                // Short rumble feedback to confirm toggle
                gamepad1.rumble(200);
            }
            lastOptionsState = gamepad1.options;

            // SHARE button (⧉) - Reset heading to 0° (redefine "forward")
            if (gamepad1.share && !lastShareState) {
                drive.resetHeading();
                // Longer rumble to distinguish from mode toggle
                gamepad1.rumble(500);
            }
            lastShareState = gamepad1.share;

            /* ========================================
             * DRIVER 1 - INTAKE CONTROLS
             * ======================================== */
/*            if (gamepad1.right_bumper) {
                intake.intakeArtifact();
            } else if (gamepad1.left_bumper) {
                intake.ejectArtifact();
            } else {
                intake.stop();
            }
*/
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

            // Alliance & Orientation telemetry
            telemetry.addLine("=== ALLIANCE & ORIENTATION ===");
            telemetry.addData("Alliance", selectedAlliance.name());
            telemetry.addData("Heading Offset", "%.0f°", selectedAlliance.getHeadingOffset());

            // Pinpoint Odometry telemetry
            telemetry.addLine();
            telemetry.addLine("=== PINPOINT ODOMETRY ===");
            telemetry.addData("Position X", "%.2f in", odometry.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
            telemetry.addData("Position Y", "%.2f in", odometry.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
            telemetry.addData("Heading", "%.1f°", odometry.getHeadingDegrees());
            telemetry.addData("Status", odometry.getDeviceStatus());
            telemetry.addData("Frequency", "%.0f Hz", odometry.getPinpointFrequency());

            // Drive telemetry
            telemetry.addLine();
            telemetry.addLine("=== DRIVE ===");
            telemetry.addData("Mode", drive.isFieldCentric() ? "FIELD-CENTRIC" : "ROBOT-CENTRIC");
            telemetry.addData("Using Pinpoint", drive.isUsingPinpoint() ? "YES" : "NO");
            telemetry.addData("Speed", String.format("%.0f%%", drive.getSpeed() * 100));
            telemetry.addData("Sensitivity", String.format("%.1fx", drive.getSensitivity()));
            telemetry.addData("Deadzone", String.format("%.2f", drive.getDeadzone()));
            telemetry.addLine();
            telemetry.addData("Left Front", "%.2f", drive.getLeftFrontPower());
            telemetry.addData("Right Front", "%.2f", drive.getRightFrontPower());
            telemetry.addData("Left Back", "%.2f", drive.getLeftBackPower());
            telemetry.addData("Right Back", "%.2f", drive.getRightBackPower());

            // Intake telemetry
/*            telemetry.addLine();
            telemetry.addLine("=== INTAKE ===");
            telemetry.addData("Speed", "%.2f", intake.getSpeed());
            telemetry.addData("Position", "%.3f", intake.getPosition());
*/
            // Shooter telemetry
//            telemetry.addLine();
//            telemetry.addLine("=== SHOOTER ===");
//            telemetry.addData("Target Power", String.format("%.0f%%", shooter.getTargetPower() * 100));
//            telemetry.addData("Motor 1", "%.2f", shooter.shooterMotor1.getPower());
//            telemetry.addData("Motor 2", "%.2f", shooter.shooterMotor2.getPower());

            // Control hints
            telemetry.addLine();
            telemetry.addLine("=== QUICK CONTROLS ===");
            telemetry.addLine("Options: Toggle Drive Mode");
            telemetry.addLine("Share: Reset Heading");

            telemetry.update();

        } // End of main loop

        /* ========================================
         * CLEANUP
         * ======================================== */
        drive.stop();
//        intake.stop();
//        shooter.stopShooter();

    } // End of runOpMode

} // End of class