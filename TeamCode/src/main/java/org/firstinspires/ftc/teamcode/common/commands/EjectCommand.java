package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.subsystems.IntakeSubsystem;

/**
 * Runs the intake in reverse (eject) until interrupted.
 *
 * <p>Lifecycle:
 * <ol>
 *   <li>{@link #initialize()} calls {@link IntakeSubsystem#eject()}, which reverses the wheels
 *       and extends the slides.</li>
 *   <li>{@link #execute()} does nothing; {@link IntakeSubsystem#periodic()} handles
 *       auto-coupling (extend/retract) and the {@code FINISHING} stop-delay window.</li>
 *   <li>{@link #end(boolean)} calls {@link IntakeSubsystem#stop()}, which retracts the slides
 *       and arms the {@code FINISHING} timer. The command returns immediately; the subsystem
 *       transitions through {@code FINISHING} on its own without blocking the command.</li>
 * </ol>
 *
 * <p>{@link #isFinished()} always returns {@code false}: this command is intended for a
 * {@code whileTrue} binding and is ended by the trigger when the button is released, not
 * by its own termination condition.
 *
 * <p>Releasing the button does <em>not</em> block until the intake has fully stopped.
 * {@code end} returns immediately; the subsystem's {@code periodic} clears the
 * {@code FINISHING} state after the stop-delay window elapses.
 *
 * <p>Intended binding:
 * <pre>
 *   driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
 *       .and(driverGamepad.getGamepadButton(GamepadKeys.Button.A))
 *       .whileTrue(new EjectCommand(intakeSubsystem));
 * </pre>
 *
 * <p>Note: the existing TeleOp uses {@code RIGHT_BUMPER + A} as the eject combo (not
 * {@code LEFT_BUMPER}, which drives manual index-forward in the existing code). The binding
 * above carries this pairing forward into the command-based rewrite.
 */
public class EjectCommand extends CommandBase {

    private final IntakeSubsystem intake;

    public EjectCommand(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.eject();
    }

    @Override
    public void execute() {
        // IntakeSubsystem#periodic() handles auto-coupling and the FINISHING stop-delay window.
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
