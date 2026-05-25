package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.ShooterSubsystem;

/**
 * Reverses both the shooter flywheel and the index to clear a jam, running until interrupted.
 *
 * <p>Lifecycle:
 * <ol>
 *   <li>{@link #initialize()} calls {@link ShooterSubsystem#reverseFlywheel()} and
 *       {@link IndexSubsystem#reverse()}, putting both subsystems into their
 *       {@code REVERSING} state.</li>
 *   <li>{@link #execute()} does nothing; the subsystems update themselves in
 *       {@code periodic()}.</li>
 *   <li>{@link #end(boolean)} calls {@link ShooterSubsystem#stopFlywheel()} and
 *       {@link IndexSubsystem#stop()}, returning both subsystems to {@code IDLE}.</li>
 * </ol>
 *
 * <p>{@link #isFinished()} always returns {@code false}: this command is intended for a
 * {@code whileTrue} binding and is ended by the trigger when the combo is released, not
 * by its own termination condition.
 *
 * <p>Both {@code shooter} and {@code index} are declared as requirements via
 * {@code addRequirements(shooter, index)}. This is mandatory: if only one subsystem were
 * declared, the scheduler could allow a competing command (e.g. {@code ShootCommand}) to
 * claim the undeclared subsystem mid-jam-clear, producing silent, broken behavior where the
 * robot appears to jam-clear but is actually also attempting to shoot or feed.
 *
 * <p>A companion command {@code ClearIndexJamCommand} (index reverse only, no shooter) will
 * exist in a separate file and is not part of this session.
 *
 */
public class ClearJamCommand extends CommandBase {

    private final ShooterSubsystem shooter;
    private final IndexSubsystem index;

    public ClearJamCommand(ShooterSubsystem shooter, IndexSubsystem index) {
        this.shooter = shooter;
        this.index = index;
        addRequirements(shooter, index);
    }

    @Override
    public void initialize() {
        shooter.reverseFlywheel();
        index.reverse();
    }

    @Override
    public void execute() {
        // Both subsystems update themselves in periodic(); nothing to do per tick.
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
        index.stop();
    }
}
