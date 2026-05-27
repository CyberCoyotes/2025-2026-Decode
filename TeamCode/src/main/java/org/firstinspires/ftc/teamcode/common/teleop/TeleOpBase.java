package org.firstinspires.ftc.teamcode.common.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.commands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.common.commands.EjectCommand;
import org.firstinspires.ftc.teamcode.common.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.common.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.common.config.RobotConfig;
import org.firstinspires.ftc.teamcode.common.helpers.Limelight;
import org.firstinspires.ftc.teamcode.common.helpers.PinpointOdometry;
import org.firstinspires.ftc.teamcode.common.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.ShooterSubsystem;

import static org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem.SpeedMode;

/**
 * Abstract base for all team-specific TeleOp OpModes.
 *
 * <h2>Structure</h2>
 * <ol>
 *   <li><strong>Helpers updated first</strong> — {@code pinpoint.update()} and
 *       {@code limelight.update()} fire at the top of every loop tick, before the FTCLib
 *       scheduler runs. This guarantees that the heading supplier wired into
 *       {@link MecanumDriveSubsystem} delivers fresh data within the same tick, so
 *       field-centric rotation is never one cycle stale.</li>
 *   <li><strong>Scheduler runs subsystems and commands</strong> — {@code super.run()} dispatches
 *       {@code periodic()} on each registered subsystem and advances all active commands.</li>
 *   <li><strong>Telemetry last</strong> — {@code updateTelemetry()} reads the freshly-updated
 *       state of every subsystem and helper and flushes the driver station in a single call.</li>
 * </ol>
 *
 * <h2>Gamepad configuration</h2>
 * This build is a <strong>single-driver demo configuration</strong>: all controls live on
 * {@code gamepad1}. {@code gamepad2} is unused and no field for it is declared. Flavor classes
 * that extend this base should bind all commands in their {@link #bindCommands()} override using
 * {@code driverGamepad} only.
 *
 * <h2>Session structure</h2>
 * <ul>
 *   <li><strong>Session A (this file)</strong> — shell: subsystem + helper construction,
 *       default drive command, helper update loop. No trigger bindings.</li>
 *   <li><strong>Session B</strong> — populates {@link #bindCommands()} with trigger bindings
 *       for all current command classes: {@code IntakeCommand}, {@code EjectCommand},
 *       {@code ShootCommand}, {@code ClearJamCommand}, {@code ClearIndexJamCommand},
 *       and drive-mode / speed-mode instant commands via {@code DefaultDriveCommand}.</li>
 *   <li><strong>Session C</strong> — updates the team flavor wrappers
 *       ({@code team11940/PrimeTeleOp.java}, {@code team22091/PrimeTeleOp.java}) to extend
 *       this class.</li>
 * </ul>
 */
public abstract class TeleOpBase extends CommandOpMode {

    // ---- Subsystems (actuator hardware — registered with the scheduler) ----

    private MecanumDriveSubsystem drive;
    private IntakeSubsystem       intake;
    private IndexSubsystem        index;
    private ShooterSubsystem      shooter;

    // ---- Helpers (pure-read sensors — updated manually, NOT registered) ----

    private PinpointOdometry pinpoint;
    private Limelight        limelight;

    // ---- Gamepads ----------------------------------------------------------
    // Single-driver demo configuration: gamepad2 is unused; no field is declared for it.

    private GamepadEx driverGamepad;

    // ---- Config ------------------------------------------------------------

    private RobotConfig config;

    // ---- Abstract hook -----------------------------------------------------

    /**
     * Provide the per-robot configuration for this OpMode flavor.
     * Called once at the start of {@link #initialize()}.
     */
    protected abstract RobotConfig getRobotConfig();

    // ========================================================================
    // INITIALIZE
    // ========================================================================

    @Override
    public void initialize() {

        // 1. Config first — subsystems read defaults from it during construction.
        config = getRobotConfig();

        // 2. Gamepad wrapper.
        driverGamepad = new GamepadEx(gamepad1);

        // 3. Helpers first — drive captures pinpoint::getHeadingRadians as a live
        //    DoubleSupplier, so the helper must exist before drive is constructed.
        pinpoint  = new PinpointOdometry(hardwareMap);
        limelight = new Limelight(hardwareMap);

        // 4. Drive — wires in the heading supplier from pinpoint.
        drive = new MecanumDriveSubsystem(hardwareMap, pinpoint::getHeadingRadians, config);

        // 5. Remaining actuator subsystems.
        intake  = new IntakeSubsystem(hardwareMap);
        index   = new IndexSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);

        // 6. Register actuator subsystems so the scheduler calls their periodic() each tick.
        //    Helpers are NOT registered — they have no periodic() and are not SubsystemBase.
        register(drive, intake, index, shooter);

        // 7. Default drive command — runs whenever no other command claims the drive subsystem.
        drive.setDefaultCommand(new DefaultDriveCommand(drive, driverGamepad));

        // 8. Initial drive state: field-centric from match start, heading zeroed here.
        drive.setHeadingMode(MecanumDriveSubsystem.HeadingMode.FIELD_CENTRIC);
        drive.resetHeading();

        // 9. Signal to the drive station that init is complete.
        telemetry.addData("Status", "Ready");
        telemetry.update();

        // 10. Trigger bindings — populated by Session B.
        bindCommands();
    }

    // ========================================================================
    // RUN LOOP
    // ========================================================================

    /**
     * Main loop tick.
     *
     * <p>Helper updates fire <em>before</em> {@code super.run()} so that
     * {@link MecanumDriveSubsystem#periodic()} consumes fresh heading data within the
     * same tick. Telemetry is flushed last, after all state has settled.
     */
    @Override
    public void run() {
        pinpoint.update();   // pull fresh pose + heading from Pinpoint over I2C
        limelight.update();  // pull latest result + status snapshot from Limelight

        super.run();         // scheduler: subsystem periodic() + active commands

        updateTelemetry();
    }

    // ========================================================================
    // BIND COMMANDS
    // ========================================================================

    /**
     * Override in a subclass to wire trigger bindings.
     *
     * <p>Called once at the end of {@link #initialize()}, after subsystems, helpers,
     * and the default drive command are fully constructed. Bindings declared here can
     * safely reference any field on this class.
     */
    protected void bindCommands() {

        // ---- Drive: precision speed mode ------------------------------------
        // Left trigger held → PRECISION; releasing returns to NORMAL.
        // whileFalse ensures NORMAL is restored even if the trigger isn't pressed at startup.
        new Trigger(() -> driverGamepad.getLeftTrigger() > 0.1)
                .whileTrue(new RunCommand(() -> drive.setSpeedMode(SpeedMode.PRECISION), drive))
                .whileFalse(new RunCommand(() -> drive.setSpeedMode(SpeedMode.NORMAL), drive));

        // ---- Intake / eject -------------------------------------------------
        // Left bumper held → run intake wheels forward + extend slides.
        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileTrue(new IntakeCommand(intake));

        // X held → run intake wheels in reverse (eject).
        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whileTrue(new EjectCommand(intake));

        // ---- Shooting -------------------------------------------------------
        // Right bumper held → spin up + feed when at speed.
        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whileTrue(new ShootCommand(shooter, index));

        // Right trigger pressed → same shoot sequence (separate binding; easy to reassign later).
        new Trigger(() -> driverGamepad.getRightTrigger() > 0.1)
                .whileTrue(new ShootCommand(shooter, index));

        // ---- Preset selection -----------------------------------------------
        // A → SHORT_RANGE preset (2 200 RPM, hood 0.30).
        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                .onTrue(new InstantCommand(
                        () -> shooter.setPreset(ShooterSubsystem.ShotPreset.SHORT_RANGE), shooter));

        // B → MEDIUM_RANGE preset (2 500 RPM, hood 0.60).
        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .onTrue(new InstantCommand(
                        () -> shooter.setPreset(ShooterSubsystem.ShotPreset.MEDIUM_RANGE), shooter));

        // Y → LONG_RANGE preset (2 800 RPM, hood 0.60).
        driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .onTrue(new InstantCommand(
                        () -> shooter.setPreset(ShooterSubsystem.ShotPreset.LONG_RANGE), shooter));

        // ---- Heading / drive mode -------------------------------------------
        // BACK → switch to field-centric heading mode.
        driverGamepad.getGamepadButton(GamepadKeys.Button.BACK)
                .onTrue(new InstantCommand(
                        () -> drive.setHeadingMode(MecanumDriveSubsystem.HeadingMode.FIELD_CENTRIC),
                        drive));

        // GUIDE → zero the field-centric heading reference.
        driverGamepad.getGamepadButton(GamepadKeys.Button.GUIDE)
                .onTrue(new InstantCommand(() -> drive.resetHeading(), drive));

        // ---- Configuration adjustments --------------------------------------
        // D-pad up/down: base speed ±0.05, clamped [0.1, 1.0].
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .onTrue(new InstantCommand(
                        () -> drive.setBaseSpeed(Math.min(1.0, drive.getBaseSpeed() + 0.05)), drive));

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .onTrue(new InstantCommand(
                        () -> drive.setBaseSpeed(Math.max(0.1, drive.getBaseSpeed() - 0.05)), drive));

        // D-pad right/left: sensitivity ±0.1, clamped [0.5, 2.0].
        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .onTrue(new InstantCommand(
                        () -> drive.setSensitivity(Math.min(2.0, drive.getSensitivity() + 0.1)),
                        drive));

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .onTrue(new InstantCommand(
                        () -> drive.setSensitivity(Math.max(0.5, drive.getSensitivity() - 0.1)),
                        drive));
    }

    // ========================================================================
    // TELEMETRY
    // ========================================================================

    /**
     * Emit one driver-station screen worth of status from all subsystems and helpers.
     *
     * <p>Called at the end of every {@link #run()} tick. All formatting is delegated to
     * {@code telemetry.addData(key, format, args)} to avoid per-frame string allocation.
     * Aim: ~12 lines, no scrolling required on a standard driver-station display.
     */
    protected void updateTelemetry() {

        // ---- Drive ---------------------------------------------------------
        telemetry.addData("Drive",
                "%s | %s | spd=%.2f | sens=%.1f",
                drive.getHeadingMode().name(),
                drive.getSpeedMode().name(),
                drive.getBaseSpeed(),
                drive.getSensitivity());

        telemetry.addData("Heading",
                "%.1f deg",
                Math.toDegrees(drive.getHeading()));

        // ---- Odometry ------------------------------------------------------
        telemetry.addData("Position",
                "X=%.2f\"  Y=%.2f\"",
                pinpoint.getX(DistanceUnit.INCH),
                pinpoint.getY(DistanceUnit.INCH));

        telemetry.addData("Pinpoint",
                "%s  %.0f Hz",
                pinpoint.getDeviceStatus(),
                pinpoint.getPinpointFrequency());

        // ---- Vision --------------------------------------------------------
        if (limelight.hasTarget()) {
            telemetry.addData("Limelight",
                    "%s  tx=%.1f  ty=%.1f",
                    limelight.getStatus(),
                    limelight.getTx(),
                    limelight.getTy());
        } else {
            telemetry.addData("Limelight", limelight.getStatus().name());
        }

        // ---- Intake --------------------------------------------------------
        telemetry.addData("Intake",
                "%s | slides=%s",
                intake.getStatus().name(),
                intake.isExtended() ? "OUT" : "IN");

        // ---- Index ---------------------------------------------------------
        telemetry.addData("Index",
                "%s | artifact=%s",
                index.getStatus().name(),
                index.hasArtifact() ? "YES" : "no");

        // ---- Shooter -------------------------------------------------------
        telemetry.addData("Shooter",
                "%s%s",
                shooter.getStatus().name(),
                shooter.isReady() ? " [READY]" : "");

        telemetry.addData("Preset",
                "%s  tgt=%.0f  act=%.0f RPM",
                shooter.getPreset().name(),
                shooter.getTargetRPM(),
                shooter.getCurrentRPM());

        telemetry.addData("Hood", "%.2f", shooter.getHoodPosition());

        telemetry.update();
    }
}
