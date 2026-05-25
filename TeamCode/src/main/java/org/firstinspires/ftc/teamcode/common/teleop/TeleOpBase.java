package org.firstinspires.ftc.teamcode.common.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.common.config.RobotConfig;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem.DriveState;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.PinpointOdometrySubsystem;

public abstract class PrimeTeleOpBase extends LinearOpMode {

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
    private boolean lastBackState = false;
    private boolean lastStartState = false;
    private boolean lastGuideState = false;
    private boolean lastDpadUpState = false;
    private boolean lastDpadDownState = false;
    private boolean lastDpadLeftState = false;
    private boolean lastDpadRightState = false;

    // Gamepad 2 - Shooter controls
    private boolean lastGP2_XState = false;
    private boolean lastGP2_YState = false;
    private boolean lastGP2_BState = false;
    private boolean lastGP2_DpadUpState = false;
    private boolean lastGP2_DpadDownState = false;
    private boolean lastGP2_DpadLeftState = false;
    private boolean lastGP2_DpadRightState = false;

    // Flywheel and index sequential control state
    private boolean flywheelRunning = false;

    // Index motor delayed stop state
    private boolean indexBottomDelayedStop = false;
    private long indexBottomStopTime = 0;
    private boolean indexTopDelayedStop = false;
    private long indexTopStopTime = 0;

    private boolean manualIndexingActive = false;
    private boolean intakeIndexMotorsStarted = false;

    /* ========================================
     * CONSTANTS
     * ======================================== */
    private static final double SPEED_STEP = 0.05;
    private static final double SENSITIVITY_STEP = 0.1;
    private static final double SLOW_MOTION_SPEED = 0.30;
    private static final double TURBO_SPEED = 1.0;
    private static final double SHOOTER_POWER_INCREMENT = 0.05;
    private static final double SHOOTER_MIN_POWER = 0.0;
    private static final double SHOOTER_MAX_POWER = 1.0;
    private static final double HOOD_INCREMENT = 0.05;
    private static final long INDEX_BOTTOM_STOP_DELAY_MS = 1500;
    private static final long INDEX_TOP_STOP_DELAY_MS = 1500;

    /* ========================================
     * CONFIGURATION VARIABLES
     * ======================================== */
    private double baseSpeed;
    private double shooterPower = 0.50;
    private RobotConfig config;

    protected abstract RobotConfig getRobotConfig();

    @Override
    public void runOpMode() throws InterruptedException {

        config = getRobotConfig();
        baseSpeed = config.defaultBaseSpeed();

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        drive = new MecanumDriveSubsystem(hardwareMap);
        odometry = new PinpointOdometrySubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        index = new IndexSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        drive.setOdometry(odometry);
        drive.setSensitivity(config.defaultSensitivity());
        drive.setDeadzone(config.defaultDeadzone());

        telemetry.addData("Status", "Ready to start!");
        telemetry.addLine();
        telemetry.addLine("=== GAMEPAD 1 (DRIVER) ===");
        telemetry.addLine("  Left Stick      - Strafe");
        telemetry.addLine("  Right Stick     - Rotate");
        telemetry.addLine("  Left Trigger    - Slow Motion (30% Speed)");
        telemetry.addLine("  Right Trigger   - Turbo Mode (100% Speed)");
        telemetry.addLine("  Left Bumper     - Index Motors Only");
        telemetry.addLine("  Right Bumper    - Intake (Wheels + Index)");
        telemetry.addLine("  RB + A Button   - Eject (Reverse Intake)");
        telemetry.addLine("  A Button (alone) - Emergency Stop");
        telemetry.addLine("  BACK Button     - Toggle Field-Centric");
        telemetry.addLine("  START Button    - Toggle Robot-Centric");
        telemetry.addLine("  MODE Button     - Reset Heading");
        telemetry.addLine("  D-Pad Up/Down   - Adjust Speed");
        telemetry.addLine("  D-Pad Left/Right - Adjust Sensitivity");
        telemetry.addLine();
        telemetry.addLine("=== GAMEPAD 2 (OPERATOR) ===");
        telemetry.addLine("  Left Bumper     - Run Index Motor Forward");
        telemetry.addLine("  LB + A Button   - Reverse Index Only");
        telemetry.addLine("  Right Bumper    - Shoot at Preset (Flywheel→Index)");
        telemetry.addLine("  RB + A Button   - Reverse Flywheel & Index");
        telemetry.addLine("  X Button        - SHORT_RANGE Preset (RPM+Hood)");
        telemetry.addLine("  Y Button        - MEDIUM_RANGE Preset (RPM+Hood)");
        telemetry.addLine("  B Button        - LONG_RANGE Preset (RPM+Hood)");
        telemetry.addLine("  D-Pad Left      - Hood Servo Down (Manual)");
        telemetry.addLine("  D-Pad Right     - Hood Servo Up (Manual)");
        telemetry.addLine("  D-Pad Up/Down   - Adjust Shooter RPM (±100)");
        telemetry.update();

        waitForStart();

        drive.setState(DriveState.FIELD_CENTRIC);
        drive.resetHeading();

        while (opModeIsActive()) {

            odometry.update();
            intake.periodic();
            shooter.periodic();

            handleStateTransitions();
            handleConfigurationAdjustments();
            handleIntakeControls();

            if (indexBottomDelayedStop && System.currentTimeMillis() >= indexBottomStopTime) {
                index.stopBottomMotor();
                indexBottomDelayedStop = false;
            }
            if (indexTopDelayedStop && System.currentTimeMillis() >= indexTopStopTime) {
                index.stopTopMotor();
                indexTopDelayedStop = false;
            }

            handleIndexControls();

            index.setFlywheelOverride(flywheelRunning);
            index.periodic();

            handleShooterControls();

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double effectiveSpeed = baseSpeed;
            if (gamepad1.left_trigger > 0.1) {
                double targetSpeed = baseSpeed * SLOW_MOTION_SPEED;
                effectiveSpeed = baseSpeed + (targetSpeed - baseSpeed) * gamepad1.left_trigger;
            } else if (gamepad1.right_trigger > 0.1) {
                effectiveSpeed = baseSpeed + (TURBO_SPEED - baseSpeed) * gamepad1.right_trigger;
            }

            drive.setSpeed(effectiveSpeed);

            if (drive.getState() != DriveState.IDLE) {
                drive.drive(axial, lateral, yaw);
            }

            updateTelemetry();
        }
    }

    /* ========================================
     * STATE TRANSITION HANDLER
     * ======================================== */
    private void handleStateTransitions() {
        DriveState currentState = drive.getState();

        if (gamepad1.back && !lastBackState) {
            drive.setState(DriveState.FIELD_CENTRIC);
        }
        lastBackState = gamepad1.back;

        if (gamepad1.start && !lastStartState) {
            drive.setState(DriveState.ROBOT_CENTRIC);
        }
        lastStartState = gamepad1.start;

        if (gamepad1.guide && !lastGuideState) {
            if (currentState == DriveState.FIELD_CENTRIC) {
                drive.resetHeading();
                telemetry.addData("Action", "Heading Reset!");
            }
        }
        lastGuideState = gamepad1.guide;

        if (gamepad1.a && !gamepad1.right_bumper && !gamepad1.left_bumper) {
            drive.setState(DriveState.IDLE);
        } else if (currentState == DriveState.IDLE && !gamepad1.a) {
            drive.setState(DriveState.ROBOT_CENTRIC);
        }
    }

    /* ========================================
     * CONFIGURATION ADJUSTMENT HANDLER
     * ======================================== */
    private void handleConfigurationAdjustments() {
        if (gamepad1.dpad_up && !lastDpadUpState) {
            baseSpeed = Math.min(1.0, baseSpeed + SPEED_STEP);
        }
        lastDpadUpState = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !lastDpadDownState) {
            baseSpeed = Math.max(0.1, baseSpeed - SPEED_STEP);
        }
        lastDpadDownState = gamepad1.dpad_down;

        if (gamepad1.dpad_right && !lastDpadRightState) {
            drive.setSensitivity(Math.min(2.0, drive.getSensitivity() + SENSITIVITY_STEP));
        }
        lastDpadRightState = gamepad1.dpad_right;

        if (gamepad1.dpad_left && !lastDpadLeftState) {
            drive.setSensitivity(Math.max(0.5, drive.getSensitivity() - SENSITIVITY_STEP));
        }
        lastDpadLeftState = gamepad1.dpad_left;
    }

    /* ========================================
     * INTAKE CONTROL HANDLER
     * ======================================== */
    private void handleIntakeControls() {
        if (gamepad1.left_bumper && gamepad1.a) {
            index.runReverse();
            manualIndexingActive = true;
            indexBottomDelayedStop = false;
            indexTopDelayedStop = false;
        } else if (gamepad1.left_bumper) {
            if (!manualIndexingActive) {
                index.runBottomMotorForward();
                index.runTopMotorForward();
                manualIndexingActive = true;
            }
            indexBottomDelayedStop = false;
            indexTopDelayedStop = false;
        } else if (gamepad1.right_bumper && gamepad1.a) {
            intake.ejectArtifact();
            index.runReverse();
            indexBottomDelayedStop = false;
            indexTopDelayedStop = false;
            intakeIndexMotorsStarted = false;
            manualIndexingActive = false;
        } else if (gamepad1.right_bumper) {
            intake.intakeArtifact();
            if (!intakeIndexMotorsStarted) {
                index.runBottomMotorForward();
                index.runTopMotorForward();
                intakeIndexMotorsStarted = true;
            }
            indexBottomDelayedStop = false;
            indexTopDelayedStop = false;
            manualIndexingActive = false;
        } else if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
            intake.stop();
            intakeIndexMotorsStarted = false;

            if (manualIndexingActive) {
                index.stop();
                manualIndexingActive = false;
            } else {
                if (!indexBottomDelayedStop) {
                    indexBottomDelayedStop = true;
                    indexBottomStopTime = System.currentTimeMillis() + INDEX_BOTTOM_STOP_DELAY_MS;
                }
                if (!indexTopDelayedStop) {
                    indexTopDelayedStop = true;
                    indexTopStopTime = System.currentTimeMillis() + INDEX_TOP_STOP_DELAY_MS;
                }
            }
        }
    }

    /* ========================================
     * INDEX CONTROL HANDLER
     * ======================================== */
    private void handleIndexControls() {
        if (gamepad2.left_bumper && gamepad2.a) {
            index.runReverse();
            manualIndexingActive = true;
            indexBottomDelayedStop = false;
            indexTopDelayedStop = false;
        } else if (gamepad2.left_bumper) {
            if (!manualIndexingActive) {
                index.runForward();
                manualIndexingActive = true;
            }
            indexBottomDelayedStop = false;
            indexTopDelayedStop = false;
        } else if (manualIndexingActive) {
            if (!gamepad2.right_bumper) {
                index.stop();
            }
            manualIndexingActive = false;
        } else if (!gamepad2.right_bumper && !gamepad1.right_bumper && !indexBottomDelayedStop && !indexTopDelayedStop) {
            index.stop();
        }
    }

    /* ========================================
     * SHOOTER CONTROL HANDLER (GAMEPAD 2)
     * ======================================== */
    private void handleShooterControls() {
        if (gamepad2.x && !lastGP2_XState) {
            shooter.setPreset(ShooterSubsystem.ShotPreset.SHORT_RANGE);
        }
        lastGP2_XState = gamepad2.x;

        if (gamepad2.y && !lastGP2_YState) {
            shooter.setPreset(ShooterSubsystem.ShotPreset.MEDIUM_RANGE);
        }
        lastGP2_YState = gamepad2.y;

        if (gamepad2.b && !lastGP2_BState) {
            shooter.setPreset(ShooterSubsystem.ShotPreset.LONG_RANGE);
        }
        lastGP2_BState = gamepad2.b;

        if (gamepad2.dpad_up && !lastGP2_DpadUpState) {
            shooter.incrementFlywheelRPM();
        }
        lastGP2_DpadUpState = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastGP2_DpadDownState) {
            shooter.decrementFlywheelRPM();
        }
        lastGP2_DpadDownState = gamepad2.dpad_down;

        if (gamepad2.dpad_left && !lastGP2_DpadLeftState) {
            shooter.setHoodPosition(shooter.getHoodPosition() - HOOD_INCREMENT);
        }
        lastGP2_DpadLeftState = gamepad2.dpad_left;

        if (gamepad2.dpad_right && !lastGP2_DpadRightState) {
            shooter.setHoodPosition(shooter.getHoodPosition() + HOOD_INCREMENT);
        }
        lastGP2_DpadRightState = gamepad2.dpad_right;

        if (gamepad2.right_bumper && gamepad2.a) {
            shooter.runFlywheelReverse();
            index.runReverse();
            flywheelRunning = true;
        } else if (gamepad2.right_bumper) {
            if (!flywheelRunning) {
                shooter.setPreset(shooter.getPreset());
                flywheelRunning = true;
            }
            if (shooter.isAtTargetRPM()) {
                index.runForward();
            }
        } else {
            if (flywheelRunning) {
                shooter.stopFlywheel();
                if (!gamepad1.right_bumper) {
                    index.stop();
                }
                flywheelRunning = false;
            }
        }
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
        telemetry.addData("Bottom Motor", "%.2f (%.0f%%)", index.getBottomMotorPower(), index.getBottomMotorPower() * 100);
        telemetry.addData("Top Motor", "%.2f (%.0f%%)", index.getTopMotorPower(), index.getTopMotorPower() * 100);
        telemetry.addLine();
        telemetry.addData("Sensor Distance", "%.2f cm", index.getDistance());
        telemetry.addData("Threshold", "%.2f cm", 3.0);
        telemetry.addData("Artifact Detected", index.isArtifactDetected() ? "✓ YES (< 3cm)" : "✗ NO (>= 3cm)");
        telemetry.addLine();
        telemetry.addData("Flywheel Override", index.isFlywheelOverrideActive() ? "ACTIVE" : "DISABLED");
        telemetry.addData("Manual Index", manualIndexingActive ? "ACTIVE" : "IDLE");
        telemetry.addData("Intake Started", intakeIndexMotorsStarted ? "YES" : "NO");
        telemetry.addData("Bottom Delayed Stop", indexBottomDelayedStop ? String.format("PENDING (%d ms)", indexBottomStopTime - System.currentTimeMillis()) : "NONE");
        telemetry.addData("Top Delayed Stop", indexTopDelayedStop ? String.format("PENDING (%d ms)", indexTopStopTime - System.currentTimeMillis()) : "NONE");

        telemetry.addLine();
        telemetry.addLine("=== SHOOTER SUBSYSTEM ===");
        telemetry.addData("Preset", shooter.getPreset().name());
        telemetry.addData("Target RPM", shooter.getTargetRPM());
        telemetry.addData("Current RPM", String.format("%.0f", shooter.getCurrentRPM()));
        telemetry.addData("At Target", shooter.isAtTargetRPM() ? "✓ YES" : "✗ NO");
        telemetry.addData("Motor Power", String.format("%.0f%%", shooter.getFlywheelPower() * 100));
        telemetry.addData("Preset Hood", String.format("%.2f", shooter.getPreset().getHoodPosition()));
        telemetry.addData("Current Hood", String.format("%.2f", shooter.getHoodPosition()));
        telemetry.addData("Sequential Mode", flywheelRunning ? "RUNNING" : "IDLE");

        telemetry.addLine();
        telemetry.addLine("=== CONFIGURATION ===");
        telemetry.addData("Sensitivity", "%.1f", drive.getSensitivity());
        telemetry.addData("Deadzone", "%.2f", config.defaultDeadzone());

        telemetry.addLine();
        telemetry.addLine("=== MODES ===");
        telemetry.addData("Field-Centric", drive.getState() == DriveState.FIELD_CENTRIC ? "ON" : "OFF");
        telemetry.addData("Robot-Centric", drive.getState() == DriveState.ROBOT_CENTRIC ? "ON" : "OFF");

        String speedMode = "Normal";
        if (gamepad1.left_trigger > 0.1) {
            speedMode = "SLOW MOTION (30%)";
        } else if (gamepad1.right_trigger > 0.1) {
            speedMode = "TURBO (100%)";
        }
        telemetry.addData("Speed Mode", speedMode);

        telemetry.update();
    }
}
