package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.common.subsystems.MecanumDriveSubsystem;

/**
 * Default command for {@link MecanumDriveSubsystem}.
 *
 * <p>A <em>default command</em> is the command the scheduler runs on a subsystem whenever no
 * other command is using it. It is registered via
 * {@code drive.setDefaultCommand(new DefaultDriveCommand(drive, driverGamepad))} in the TeleOp
 * setup method and starts running automatically at the beginning of the OpMode.
 *
 * <p>Lifecycle relative to other drive commands:
 * <ul>
 *   <li>When an auto-align (or any other command that requires the drive subsystem) is triggered,
 *       the scheduler suspends this command by calling {@link #end(boolean)} with
 *       {@code interrupted = true}, then hands control to the new command.</li>
 *   <li>When that other command finishes or is cancelled, the scheduler automatically restarts
 *       this default command from {@link #initialize()}. The driver regains stick control without
 *       any explicit re-binding. This automatic hand-back is the defining property of the default
 *       command pattern.</li>
 * </ul>
 *
 * <p>Responsibilities of this class:
 * <ul>
 *   <li>Read the driver gamepad sticks each tick.</li>
 *   <li>Forward the raw values to {@link MecanumDriveSubsystem#drive(double, double, double)}.</li>
 *   <li>Call {@link MecanumDriveSubsystem#stop()} when interrupted.</li>
 * </ul>
 *
 * <p>Responsibilities that live <em>elsewhere</em>:
 * <ul>
 *   <li><strong>Deadzone, sensitivity curve, speed multipliers, field-centric rotation, mecanum
 *       mixing:</strong> all handled inside {@link MecanumDriveSubsystem#periodic()}. This command
 *       passes raw stick values straight through — no transformations here.</li>
 *   <li><strong>Heading mode toggle, speed mode toggle, heading reset, base-speed adjustment,
 *       emergency stop:</strong> bound as separate {@code InstantCommand} / {@code RunCommand}
 *       trigger bindings in the TeleOp class. They are not handled here.</li>
 * </ul>
 */
public class DefaultDriveCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final GamepadEx driverGamepad;

    public DefaultDriveCommand(MecanumDriveSubsystem drive, GamepadEx driverGamepad) {
        this.drive = drive;
        this.driverGamepad = driverGamepad;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Default commands typically have no one-time setup; the subsystem is already running.
    }

    @Override
    public void execute() {
        double forward = -driverGamepad.getLeftY();  // gamepad Y is inverted: up = negative
        double strafe  =  driverGamepad.getLeftX();
        double rotate  =  driverGamepad.getRightX();
        drive.drive(forward, strafe, rotate);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
