package org.firstinspires.ftc.teamcode.team22091;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.pedropathing.PedroAutonBase;

/**
 * Team 22091 primary autonomous.
 *
 * COORDINATE SYSTEM (Pedro Pathing):
 *   - Field is 144" × 144". (0, 0) = bottom-left corner of field.
 *   - X increases toward the right wall, Y increases toward the far wall.
 *   - Heading 0 = facing right (+X), 90° = facing far wall (+Y).
 *   - Use https://visualizer.pedropathing.com/ to visualize paths.
 *
 * STARTING POSITION:
 *   Update START_POSE after you know the game field layout and where
 *   the robot will be placed at the start of each match.
 *
 * HOW TO ADD A SCORING ACTION:
 *   1. Initialize your subsystem in the constructor or in buildPaths().
 *   2. Call subsystem methods inside autonomousPathUpdate() at the
 *      appropriate state, interleaved with follower.followPath() calls.
 */
@Autonomous(name = "Team 22091 Auton", group = "Auton")
public class PrimeAuton extends PedroAutonBase {

    // ─── Field positions ─────────────────────────────────────────────────────
    // All units in inches. Heading in radians (use Math.toRadians()).
    // TODO: Update these after measuring your actual starting position.
    private static final Pose START_POSE = new Pose(9,  63, Math.toRadians(90));
    private static final Pose SCORE_POSE = new Pose(9,  84, Math.toRadians(90));
    private static final Pose PARK_POSE  = new Pose(36, 84, Math.toRadians(90));

    // ─── Paths ───────────────────────────────────────────────────────────────
    private PathChain toScore;
    private PathChain toPark;

    @Override
    protected Pose getStartPose() {
        return START_POSE;
    }

    @Override
    protected void buildPaths() {
        toScore = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, SCORE_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        toPark = follower.pathBuilder()
                .addPath(new BezierLine(SCORE_POSE, PARK_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    // ─── State machine ───────────────────────────────────────────────────────
    // States:
    //   0  — drive to scoring position
    //   1  — wait to arrive, then perform scoring action
    //   2  — drive to park
    //   3  — wait to arrive, done
    //  -1  — finished

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(toScore, /* holdEnd= */ true);
                setPathState(1);
                break;

            case 1:
                if (isNear(SCORE_POSE, 3)) {
                    // TODO: trigger scoring action here
                    //   e.g. shooter.spinUp(); index.fire();
                    follower.followPath(toPark, /* holdEnd= */ true);
                    setPathState(2);
                }
                // Safety timeout — if we haven't arrived in 4 seconds, move on anyway
                if (stateTime() > 4.0) {
                    follower.followPath(toPark, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }
}
