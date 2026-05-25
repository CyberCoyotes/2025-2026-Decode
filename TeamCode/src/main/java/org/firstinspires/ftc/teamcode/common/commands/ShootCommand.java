package org.firstinspires.ftc.teamcode.common.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystems.ShooterSubsystem;

/**
 * Orchestrates the four-step shoot sequence:
 *
 * <ol>
 *   <li><b>Spin-up:</b> {@link #initialize()} re-issues the currently-selected preset to the
 *       flywheel, starting the spin-up. The index is not touched yet.</li>
 *   <li><b>Wait:</b> {@link #execute()} polls on every scheduler tick. It waits until
 *       {@link ShooterSubsystem#isReady()} returns {@code true} (flywheel within 5% of target)
 *       <em>or</em> the 2-second spin-up timeout elapses, whichever comes first.</li>
 *   <li><b>Feed:</b> Once the wait condition is satisfied, {@link IndexSubsystem#feed()} is called
 *       and a one-way latch ({@code feeding}) is set. The index feeds continuously for the
 *       remainder of the command. The latch prevents the spin-up wait from re-evaluating on
 *       subsequent ticks.</li>
 *   <li><b>End:</b> {@link #isFinished()} always returns {@code false}; the command is intended
 *       for a {@code whileTrue} binding and runs for as long as the operator holds the trigger.
 *       On release, the scheduler calls {@link #end(boolean)}, which stops the flywheel and then
 *       stops the index, returning both subsystems to {@code IDLE}.</li>
 * </ol>
 *
 * <p>The 2-second timeout is PIDF failure protection. If the flywheel never converges to
 * {@code AT_SPEED} — due to a motor fault, encoder glitch, or aggressive tuning — the operator
 * still gets to fire rather than waiting forever for a condition that will never arrive.
 * The timeout governs only the spin-up wait; there is no overall time limit on the command itself.
 *
 * <p>This command does not pick a preset. Preset selection is a separate operator action handled
 * in TeleOp. This command re-issues whatever preset is currently active at the moment the trigger
 * fires, ensuring the flywheel is actually commanded even if it had coasted to idle since the
 * last preset selection.
 *
 * <p>Both {@link ShooterSubsystem} and {@link IndexSubsystem} are declared as requirements.
 * This prevents the scheduler from running {@link ClearJamCommand} or any other command that
 * touches either subsystem concurrently.
 */
public class ShootCommand extends CommandBase {

    private static final double SPIN_UP_TIMEOUT_SECONDS = 2.0;

    private final ShooterSubsystem shooter;
    private final IndexSubsystem   index;
    private final ElapsedTime      spinUpTimer;

    private boolean feeding;

    public ShootCommand(ShooterSubsystem shooter, IndexSubsystem index) {
        this.shooter     = shooter;
        this.index       = index;
        this.spinUpTimer = new ElapsedTime();
        addRequirements(shooter, index);
    }

    @Override
    public void initialize() {
        spinUpTimer.reset();
        feeding = false;
        shooter.setPreset(shooter.getPreset());
    }

    @Override
    public void execute() {
        if (feeding) return;
        if (shooter.isReady() || spinUpTimer.seconds() >= SPIN_UP_TIMEOUT_SECONDS) {
            index.feed();
            feeding = true;
        }
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
