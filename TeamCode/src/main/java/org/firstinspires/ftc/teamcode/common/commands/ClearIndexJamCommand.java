package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.subsystems.IndexSubsystem;

/**
 * Reverses the index only to clear an index jam, running until interrupted.
 *
 * <p>This command exists separately from {@link ClearJamCommand} because there are two
 * distinct operator scenarios: a full-system jam (shooter + index both stuck, cleared with
 * {@code ClearJamCommand} on RB&nbsp;+&nbsp;A) and an index-only jam where the shooter is
 * already idle or fine (cleared with this command on LB&nbsp;+&nbsp;A). Two commands, two
 * files, two clearly-named bindings make both scenarios independently discoverable and
 * independently adjustable.
 *
 * <p>Lifecycle:
 * <ol>
 *   <li>{@link #initialize()} calls {@link IndexSubsystem#reverse()}, putting the index into
 *       its {@code REVERSING} state.</li>
 *   <li>{@link #execute()} does nothing; the index subsystem updates itself in
 *       {@code periodic()}.</li>
 *   <li>{@link #end(boolean)} calls {@link IndexSubsystem#stop()}, returning the index to
 *       {@code IDLE}.</li>
 * </ol>
 *
 * <p>{@link #isFinished()} always returns {@code false}: this command is intended for a
 * {@code whileTrue} binding and is ended by the trigger when the combo is released, not
 * by its own termination condition.
 *
 * <p>Only {@code index} is declared as a requirement via {@code addRequirements(index)}.
 * The shooter is intentionally omitted: this command does not touch the shooter, and
 * excluding it from requirements allows a concurrent shooter command to continue
 * uninterrupted while the index jam is cleared.
 *
 * <p>Intended binding (operator gamepad, LB + A):
 * <pre>
 *   operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
 *       .and(operatorGamepad.getGamepadButton(GamepadKeys.Button.A))
 *       .whileTrue(new ClearIndexJamCommand(indexSubsystem));
 * </pre>
 */
public class ClearIndexJamCommand extends CommandBase {

    private final IndexSubsystem index;

    public ClearIndexJamCommand(IndexSubsystem index) {
        this.index = index;
        addRequirements(index);
    }

    @Override
    public void initialize() {
        index.reverse();
    }

    @Override
    public void execute() {
        // The index subsystem updates itself in periodic(); nothing to do per tick.
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        index.stop();
    }
}
