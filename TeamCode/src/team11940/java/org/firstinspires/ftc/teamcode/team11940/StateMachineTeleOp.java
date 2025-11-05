package org.firstinspires.ftc.teamcode.team11940;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem.DriveState;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem.SlideState;
import org.firstinspires.ftc.teamcode.common.subsystems.IndexSubsystem;

@TeleOp(name = "TeleOp (State Machine)", group = "TeleOp")
public class StateMachineTeleOp extends LinearOpMode {

    /* ========================================
     * SUBSYSTEMS
     * ======================================== */
    private MecanumDriveSubsystem drive;
    private IntakeSubsystem intake;
    private IndexSubsystem index;

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

    /* ========================================
     * CONSTANTS
     * ======================================== */
    private static final double DEFAULT_SENSITIVITY = 1.2;
    private static final double DEFAULT_DEADZONE = 0.1;
    private static final double SPEED_STEP = 0.05;
    private static final double SENSITIVITY_STEP = 0.1;

    /* ========================================
     * CONFIGURATION VARIABLES
     * ======================================== */
    private double baseSpeed = 0.85;           // Current base speed multiplier

    @Override
    public void runOpMode() throws InterruptedException {

        /* ========================================
         * INITIALIZATION
         * ======================================== */
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize subsystems
        drive = new MecanumDriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        index = new IndexSubsystem(hardwareMap);

        drive.setSensitivity(DEFAULT_SENSITIVITY);
        drive.setDeadzone(DEFAULT_DEADZONE);

        telemetry.addData("Status", "Ready to start!");
        telemetry.addLine();
        telemetry.addLine("Controls:");
        telemetry.addLine("  Left Stick      - Strafe");
        telemetry.addLine("  Right Stick     - Rotate");
        telemetry.addLine("  Right Bumper    - Intake Wheels");
        telemetry.addLine("  Left Bumper     - Eject Wheels");
        telemetry.addLine("  (Slides auto-extend/retract)");
        telemetry.addLine("  Y Button        - Run Index Motor Forward");
        telemetry.addLine("  X Button        - Toggle Field-Centric");
        telemetry.addLine("  B Button        - Toggle Turbo Mode");
        telemetry.addLine("  A Button        - Emergency Stop");
        telemetry.addLine("  Options         - Reset Heading");
        telemetry.addLine("  D-Pad Up/Down   - Adjust Speed");
        telemetry.addLine("  D-Pad Left/Right - Adjust Sensitivity");
        telemetry.update();

        waitForStart();

        // Start in robot-centric mode
        drive.setState(DriveState.ROBOT_CENTRIC);

        /* ========================================
         * MAIN LOOP
         * ======================================== */
        while (opModeIsActive()) {

            // Update subsystems
            intake.periodic(); // Handle automatic slide control
            index.periodic();  // Handle index motor updates

            // Handle state transitions
            handleStateTransitions();

            // Handle D-pad configuration adjustments
            handleConfigurationAdjustments();

            // Handle intake controls
            handleIntakeControls();

            // Handle index controls
            handleIndexControls();

            // Get joystick inputs
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;  // Reversed for forward
            double rx = gamepad1.right_stick_x;

            // Drive based on current state
            if (drive.getState() != DriveState.IDLE) {
                drive.drive(x, y, rx);
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
        // Y Button - Run index motor forward
        if (gamepad1.y) {
            index.runForward();
        } else {
            index.stop();
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
        telemetry.addLine("=== DRIVE STATE MACHINE ===");
        telemetry.addData("Current State", drive.getStateString());
        telemetry.addData("State Speed", "%.2f", drive.getStateSpeedMultiplier());
        telemetry.addData("Base Speed", "%.2f", baseSpeed);
        telemetry.addData("Effective Speed", "%.2f", drive.getStateSpeedMultiplier() * baseSpeed);

        if (drive.getState() == DriveState.FIELD_CENTRIC) {
            telemetry.addData("Heading", "%.1f°", drive.getHeading());
        }

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