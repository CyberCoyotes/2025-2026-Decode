package org.firstinspires.ftc.teamcode.team11940;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem.DriveState;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.PinpointOdometrySubsystem;

@TeleOp(name = "Team 11940 TeleOp", group = "TeleOp")
public class PrimeTeleOp extends LinearOpMode {

    /* ========================================
     * SUBSYSTEMS
     * ======================================== */
    private MecanumDriveSubsystem drive;
    private PinpointOdometrySubsystem odometry;
    private IntakeSubsystem intake;
    private IndexSubsystem index;
    private ShooterSubsystem shooter;

    /* ========================================
     * BUTTON STATE TRACKING
     * ======================================== */
    private boolean lastXState = false;         // Toggle field-centric
    private boolean lastYState = false;         // Toggle precision mode
    private boolean lastBState = false;         // Toggle turbo mode
    private boolean lastOptionsState = false;   // Reset heading
    private boolean lastDpadUpState = false;    // Increase speed
    private boolean lastDpadDownState = false;  // Decrease speed
    private boolean lastDpadLeftState = false;  // Decrease sensitivity
    private boolean lastDpadRightState = false; // Increase sensitivity

    // Gamepad 2 - Shooter controls
    private boolean lastGP2_XState = false;         // SHORT_RANGE preset
    private boolean lastGP2_YState = false;         // MEDIUM_RANGE preset
    private boolean lastGP2_AState = false;         // Toggle flywheel on/off
    private boolean lastGP2_BState = false;         // LONG_RANGE preset
    private boolean lastGP2_DpadUpState = false;    // Increase shooter RPM
    private boolean lastGP2_DpadDownState = false;  // Decrease shooter RPM
    private boolean lastGP2_DpadLeftState = false;  // Hood servo down
    private boolean lastGP2_DpadRightState = false; // Hood servo up

    // Flywheel and index sequential control state
    private boolean flywheelRunning = false;         // Track if flywheel is running
    private boolean flywheelTestMode = false;        // Track if flywheel is in test mode

    /* ========================================
     * CONSTANTS
     * ======================================== */
    private static final double DEFAULT_SENSITIVITY = 1.2;
    private static final double DEFAULT_DEADZONE = 0.1;
    private static final double SPEED_STEP = 0.05;
    private static final double SENSITIVITY_STEP = 0.1;

    // Shooter constants
    private static final double SHOOTER_POWER_INCREMENT = 0.05;  // 5% per button press
    private static final double SHOOTER_MIN_POWER = 0.0;
    private static final double SHOOTER_MAX_POWER = 1.0;
    private static final double HOOD_INCREMENT = 0.05;  // Hood position increment per button press

    /* ========================================
     * CONFIGURATION VARIABLES
     * ======================================== */
    private double baseSpeed = 0.85;           // Current base speed multiplier
    private double shooterPower = 0.50;        // Current shooter power level (starts at 50%)

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
        intake = new IntakeSubsystem(hardwareMap);
        index = new IndexSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        // Connect odometry to drive subsystem for heading information
        drive.setOdometry(odometry);

        drive.setSensitivity(DEFAULT_SENSITIVITY);
        drive.setDeadzone(DEFAULT_DEADZONE);

        telemetry.addData("Status", "Ready to start!");
        telemetry.addLine();
        /* TODO Take out for Shooter Testing only!
        telemetry.addLine("=== GAMEPAD 1 (DRIVER) ===");
        telemetry.addLine("  Left Stick      - Strafe");
        telemetry.addLine("  Right Stick     - Rotate");
        telemetry.addLine("  Right Bumper    - Intake Wheels");
        telemetry.addLine("  Left Bumper     - Eject Wheels");
        telemetry.addLine("  (Slides auto-extend/retract)");
        telemetry.addLine("  X Button        - Toggle Field-Centric");
        telemetry.addLine("  B Button        - Toggle Turbo Mode");
        telemetry.addLine("  A Button        - Emergency Stop");
        telemetry.addLine("  Options         - Reset Heading");
        telemetry.addLine("  D-Pad Up/Down   - Adjust Speed");
        telemetry.addLine("  D-Pad Left/Right - Adjust Sensitivity");
        telemetry.addLine();
         */

        telemetry.addLine("=== GAMEPAD 2 (OPERATOR) ===");
        telemetry.addLine("  Left Bumper     - Run Index Motor (Manual)");
        telemetry.addLine("  Right Bumper    - Shoot at Preset (Flywheel→Index)");
        telemetry.addLine("  A Button        - Toggle Flywheel On/Off");
        telemetry.addLine("  X Button        - SHORT_RANGE Preset");
        telemetry.addLine("  Y Button        - MEDIUM_RANGE Preset");
        telemetry.addLine("  B Button        - LONG_RANGE Preset");
        telemetry.addLine("  D-Pad Left      - Hood Servo Down");
        telemetry.addLine("  D-Pad Right     - Hood Servo Up");
        telemetry.addLine("  D-Pad Up/Down   - Adjust Shooter RPM (±100)");
        telemetry.update();

        waitForStart();

        // Start in field-centric mode for competition
        drive.setState(DriveState.FIELD_CENTRIC);
        drive.resetHeading();  // Reset heading at start

        /* ========================================
         * MAIN LOOP
         * ======================================== */
        while (opModeIsActive()) {

            // Update subsystems
            odometry.update();  // Update odometry position and heading
            intake.periodic(); // Handle automatic slide control
            index.periodic();  // Handle index motor updates
            shooter.periodic(); // Handle shooter subsystem updates

            // Handle state transitions
            handleStateTransitions();

            // Handle D-pad configuration adjustments
            handleConfigurationAdjustments();

            // Handle intake controls
            handleIntakeControls();

            // Handle index controls
            handleIndexControls();

            // Handle shooter controls (gamepad 2)
            handleShooterControls();

            // Get joystick inputs
            // drive.drive() expects: (axial, lateral, yaw)
            double axial = -gamepad1.left_stick_y;     // Forward/backward (reversed for forward)
            double lateral = gamepad1.left_stick_x;     // Strafe left/right
            double yaw = gamepad1.right_stick_x;        // Rotate left/right

            // Drive based on current state
            if (drive.getState() != DriveState.IDLE) {
                drive.drive(axial, lateral, yaw);  // Correct order: axial, lateral, yaw
            }

            // Update telemetry
            updateTelemetry();
        }
    }

    /* ========================================
     * STATE TRANSITION HANDLER
     * ======================================== */
    private void handleStateTransitions() {
        DriveState currentState = drive.getState();

        // X Button - Toggle between ROBOT_CENTRIC and FIELD_CENTRIC
        if (gamepad1.x && !lastXState) {
            if (currentState == DriveState.ROBOT_CENTRIC) {
                drive.setState(DriveState.FIELD_CENTRIC);
            } else if (currentState == DriveState.FIELD_CENTRIC) {
                drive.setState(DriveState.ROBOT_CENTRIC);
            }
        }
        lastXState = gamepad1.x;

        // Y Button - Now used for index motor control (see handleIndexControls())

        // B Button - Toggle TURBO mode
        if (gamepad1.b && !lastBState) {
            if (currentState == DriveState.TURBO) {
                // Return to previous mode
                drive.setState(DriveState.ROBOT_CENTRIC);
            } else {
                drive.setState(DriveState.TURBO);
            }
        }
        lastBState = gamepad1.b;

        // Options Button - Reset heading (only in field-centric)
        if (gamepad1.options && !lastOptionsState) {
            if (currentState == DriveState.FIELD_CENTRIC) {
                drive.resetHeading();
                telemetry.addData("Action", "Heading Reset!");
            }
        }
        lastOptionsState = gamepad1.options;

        // A Button - Emergency stop (sets to IDLE)
        if (gamepad1.a) {
            drive.setState(DriveState.IDLE);
        } else if (currentState == DriveState.IDLE) {
            // Resume to robot-centric when A is released
            drive.setState(DriveState.ROBOT_CENTRIC);
        }
    }

    /* ========================================
     * CONFIGURATION ADJUSTMENT HANDLER
     * ======================================== */
    private void handleConfigurationAdjustments() {
        // D-pad Up - Increase base speed
        if (gamepad1.dpad_up && !lastDpadUpState) {
            baseSpeed = Math.min(1.0, baseSpeed + SPEED_STEP);
        }
        lastDpadUpState = gamepad1.dpad_up;

        // D-pad Down - Decrease base speed
        if (gamepad1.dpad_down && !lastDpadDownState) {
            baseSpeed = Math.max(0.1, baseSpeed - SPEED_STEP);
        }
        lastDpadDownState = gamepad1.dpad_down;

        // D-pad Right - Increase sensitivity
        if (gamepad1.dpad_right && !lastDpadRightState) {
            double currentSensitivity = getCurrentSensitivity();
            drive.setSensitivity(Math.min(2.0, currentSensitivity + SENSITIVITY_STEP));
        }
        lastDpadRightState = gamepad1.dpad_right;

        // D-pad Left - Decrease sensitivity
        if (gamepad1.dpad_left && !lastDpadLeftState) {
            double currentSensitivity = getCurrentSensitivity();
            drive.setSensitivity(Math.max(0.5, currentSensitivity - SENSITIVITY_STEP));
        }
        lastDpadLeftState = gamepad1.dpad_left;
    }

    /* ========================================
     * INTAKE CONTROL HANDLER
     * ======================================== */
    private void handleIntakeControls() {
        // Intake wheel control (bumpers)
        // Slides automatically extend when intake is running and retract immediately when stopping
        // Wheels continue running for 300ms after slides retract
        if (gamepad1.right_bumper) {
            intake.intakeArtifact();
        }
        else if (gamepad1.left_bumper) {
            intake.ejectArtifact();
        }
        else {
            intake.stop();
        }

        // Manual slide override (right trigger - disabled by default, slides are automatic)
        // Uncomment below to enable manual slide control:
        // if (gamepad1.right_trigger > 0.5) {
        //     intake.disableAutoSlideControl();
        //     intake.setSlideState(SlideState.OUT);
        // } else {
        //     intake.setSlideState(SlideState.IN);
        // }
    }

    /* ========================================
     * INDEX CONTROL HANDLER
     * ======================================== */
    private void handleIndexControls() {
        // Gamepad 2 Left Bumper - Run index motor forward (manual override)
        // Note: Right bumper now handles sequential flywheel + index control
        if (gamepad2.left_bumper) {
            index.runForward();
        } else if (!gamepad2.right_bumper) {
            // Only stop if right bumper is not controlling it
            index.stop();
        }
    }

    /* ========================================
     * SHOOTER CONTROL HANDLER (GAMEPAD 2)
     * ======================================== */
    private void handleShooterControls() {
        // A Button - Toggle flywheel on/off
        if (gamepad2.a && !lastGP2_AState) {
            flywheelTestMode = !flywheelTestMode;
            if (flywheelTestMode) {
                // Turn on flywheel at current state velocity
                shooter.setFlywheelState(shooter.getFlywheelState());
            } else {
                // Turn off flywheel
                shooter.stopFlywheel();
            }
        }
        lastGP2_AState = gamepad2.a;

        // X Button - Set SHORT_RANGE preset
        if (gamepad2.x && !lastGP2_XState) {
            shooter.setFlywheelState(ShooterSubsystem.FlywheelState.SHORT_RANGE);
            // If test mode is active, apply the velocity immediately
            if (flywheelTestMode) {
                shooter.setFlywheelState(ShooterSubsystem.FlywheelState.SHORT_RANGE);
            }
        }
        lastGP2_XState = gamepad2.x;

        // Y Button - Set MEDIUM_RANGE preset
        if (gamepad2.y && !lastGP2_YState) {
            shooter.setFlywheelState(ShooterSubsystem.FlywheelState.MEDIUM_RANGE);
            // If test mode is active, apply the velocity immediately
            if (flywheelTestMode) {
                shooter.setFlywheelState(ShooterSubsystem.FlywheelState.MEDIUM_RANGE);
            }
        }
        lastGP2_YState = gamepad2.y;

        // B Button - Set LONG_RANGE preset
        if (gamepad2.b && !lastGP2_BState) {
            shooter.setFlywheelState(ShooterSubsystem.FlywheelState.LONG_RANGE);
            // If test mode is active, apply the velocity immediately
            if (flywheelTestMode) {
                shooter.setFlywheelState(ShooterSubsystem.FlywheelState.LONG_RANGE);
            }
        }
        lastGP2_BState = gamepad2.b;

        // D-pad UP increases shooter velocity by 100 RPM
        if (gamepad2.dpad_up && !lastGP2_DpadUpState) {
            shooter.incrementFlywheelRPM(100);
        }
        lastGP2_DpadUpState = gamepad2.dpad_up;

        // D-pad DOWN decreases shooter velocity by 100 RPM
        if (gamepad2.dpad_down && !lastGP2_DpadDownState) {
            shooter.decrementFlywheelRPM(100);
        }
        lastGP2_DpadDownState = gamepad2.dpad_down;

        // D-Pad Left - Decrement hood position DOWN
        if (gamepad2.dpad_left && !lastGP2_DpadLeftState) {
            double currentPosition = shooter.getHoodPosition();
            shooter.setHoodPosition(currentPosition - HOOD_INCREMENT);
        }
        lastGP2_DpadLeftState = gamepad2.dpad_left;

        // D-Pad Right - Increment hood position UP
        if (gamepad2.dpad_right && !lastGP2_DpadRightState) {
            double currentPosition = shooter.getHoodPosition();
            shooter.setHoodPosition(currentPosition + HOOD_INCREMENT);
        }
        lastGP2_DpadRightState = gamepad2.dpad_right;

        // Right Bumper - Shoot at current preset (flywheel then index)
        // Only active if NOT in test mode
        // 1. Start flywheel at current state velocity
        // 2. Wait for flywheel to reach target velocity
        // 3. Then start index motor
        // 4. Both run until button is released
        if (gamepad2.right_bumper && !flywheelTestMode) {
            // Start or continue running flywheel at current state
            if (!flywheelRunning) {
                shooter.setFlywheelState(shooter.getFlywheelState());
                flywheelRunning = true;
            }

            // Once flywheel reaches target velocity, start index motor
            if (shooter.isAtTargetVelocity()) {
                index.runForward();
            }
        } else if (!flywheelTestMode) {
            // Button released - stop both motors (only if not in test mode)
            if (flywheelRunning) {
                shooter.stopFlywheel();
                index.stop();
                flywheelRunning = false;
            }
        }
    }

    /* ========================================
     * HELPER METHODS
     * ======================================== */
    private double getCurrentSensitivity() {
        return drive.getSensitivity();
    }

    /* ========================================
     * TELEMETRY
     * ======================================== */
    private void updateTelemetry() {
        telemetry.addLine("=== PINPOINT ODOMETRY ===");
        telemetry.addData("Position X", "%.2f in", odometry.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
        telemetry.addData("Position Y", "%.2f in", odometry.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH));
        telemetry.addData("Heading", "%.1f°", odometry.getHeadingDegrees());
        telemetry.addData("Status", odometry.getDeviceStatus());
        telemetry.addData("Frequency", "%.0f Hz", odometry.getPinpointFrequency());

        telemetry.addLine();
        telemetry.addLine("=== DRIVE STATE MACHINE ===");
        telemetry.addData("Current State", drive.getStateString());
        telemetry.addData("State Speed", "%.2f", drive.getStateSpeedMultiplier());
        telemetry.addData("Base Speed", "%.2f", baseSpeed);
        telemetry.addData("Effective Speed", "%.2f", drive.getStateSpeedMultiplier() * baseSpeed);
        telemetry.addData("Using Pinpoint", drive.isUsingPinpoint() ? "YES" : "NO");

        telemetry.addLine();
        telemetry.addLine("=== INTAKE SUBSYSTEM ===");
        telemetry.addData("Wheel State", intake.getWheelStateString());
        telemetry.addData("Wheel Speed", "%.2f", intake.getSpeed());
        telemetry.addData("Slide State", intake.getSlideStateString());
        telemetry.addData("Slide Position", "%.3f", intake.getSlidePosition());
        telemetry.addData("Auto Slide Control", intake.isAutoSlideControlEnabled() ? "ENABLED" : "DISABLED");

        telemetry.addLine();
        telemetry.addLine("=== INDEX SUBSYSTEM ===");
        telemetry.addData("Index State", index.getStateString());
        telemetry.addData("Motor Power", "%.2f", index.getMotorPower());
        telemetry.addData("Motor Position", index.getMotorPosition());

        telemetry.addLine();
        telemetry.addLine("=== SHOOTER SUBSYSTEM ===");
        telemetry.addData("Test Mode", flywheelTestMode ? "ACTIVE" : "OFF");
        telemetry.addData("Flywheel State", shooter.getFlywheelState().name());
        telemetry.addData("Target RPM", shooter.getTargetRPM());
        telemetry.addData("Current RPM", String.format("%.0f",
            (shooter.getFlywheelVelocity() / 28.0) * 60.0));
        telemetry.addData("At Target", shooter.isAtTargetVelocity() ? "✓ YES" : "✗ NO");
        telemetry.addData("Velocity %", String.format("%.1f%%", shooter.getFlywheelVelocityPercentage() * 100));
        telemetry.addData("Power Setting", String.format("%.0f%% (%.2f)", shooterPower * 100, shooterPower));
        telemetry.addData("Hood Position", "%.2f", shooter.getHoodPosition());
        telemetry.addData("Sequential Mode", flywheelRunning ? "RUNNING" : "IDLE");

        telemetry.addLine();
        telemetry.addLine("=== CONFIGURATION ===");
        telemetry.addData("Sensitivity", "%.1f", getCurrentSensitivity());
        telemetry.addData("Deadzone", "%.2f", DEFAULT_DEADZONE);

        telemetry.addLine();
        telemetry.addLine("=== MODES ===");
        telemetry.addData("Field-Centric", drive.getState() == DriveState.FIELD_CENTRIC ? "ON" : "OFF");
        telemetry.addData("Precision Mode", drive.getState() == DriveState.PRECISION ? "ON" : "OFF");
        telemetry.addData("Turbo Mode", drive.getState() == DriveState.TURBO ? "ON" : "OFF");

        telemetry.update();
    }
}