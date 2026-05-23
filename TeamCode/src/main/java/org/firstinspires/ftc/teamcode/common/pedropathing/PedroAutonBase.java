package org.firstinspires.ftc.teamcode.common.pedropathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Abstract base for all Pedro Pathing autonomous OpModes.
 *
 * Subclasses provide:
 *   - getStartPose()         — where the robot is placed on the field
 *   - buildPaths()           — construct all PathChains before start
 *   - autonomousPathUpdate() — state machine called each loop iteration
 *
 * The base handles follower init, telemetry, and loop plumbing.
 * Subclasses should NOT override init/start/loop/stop.
 */
public abstract class PedroAutonBase extends OpMode {

    protected Follower follower;
    protected int pathState;

    private Timer pathTimer;

    protected abstract Pose getStartPose();
    protected abstract void buildPaths();
    protected abstract void autonomousPathUpdate();

    @Override
    public final void init() {
        pathTimer = new Timer();
        follower = PedroConfig.createFollower(hardwareMap);
        follower.setStartingPose(getStartPose());
        buildPaths();
        telemetry.addData("Status", "Ready — press Play");
        telemetry.update();
    }

    @Override
    public final void init_loop() {}

    @Override
    public final void start() {
        setPathState(0);
    }

    @Override
    public final void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("X",       "%.1f\"", follower.getPose().getX());
        telemetry.addData("Y",       "%.1f\"", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°",  Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Busy",    follower.isBusy());
        telemetry.update();
    }

    @Override
    public final void stop() {}

    // ─── Helpers ─────────────────────────────────────────────────────────────

    protected void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    /** Seconds elapsed in the current path state. Use for timed actions. */
    protected double stateTime() {
        return pathTimer.getElapsedTimeSeconds();
    }

    /** True when the robot is within {@code toleranceInches} of the target pose. */
    protected boolean isNear(Pose target, double toleranceInches) {
        Pose current = follower.getPose();
        return Math.abs(current.getX() - target.getX()) < toleranceInches
            && Math.abs(current.getY() - target.getY()) < toleranceInches;
    }
}
