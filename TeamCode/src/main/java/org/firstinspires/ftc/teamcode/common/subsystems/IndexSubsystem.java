package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IndexSubsystem extends SubsystemBase {

    // Hardware config names
    private static final String BOTTOM_MOTOR_NAME    = "indexBottomMotor";
    private static final String TOP_MOTOR_NAME       = "indexTopMotor";
    private static final String DISTANCE_SENSOR_NAME = "indexColorSensor";

    // Behavior threshold (locked — see ARCHITECTURE.md §5)
    private static final double ARTIFACT_DISTANCE_THRESHOLD_CM = 3.0;

    // Motor power levels
    private static final double FORWARD_SPEED = 1.0;
    private static final double REVERSE_SPEED = -1.0;

    public enum Status {
        IDLE,
        INTAKING,
        FEEDING,
        REVERSING
    }

    private enum Mode { IDLE, INTAKING, FEEDING, REVERSING }

    private final DcMotor        indexBottomMotor;
    private final DcMotor        indexTopMotor;
    private final DistanceSensor distanceSensor;

    private Mode mode = Mode.IDLE;

    public IndexSubsystem(HardwareMap hardwareMap) {
        indexBottomMotor = hardwareMap.get(DcMotor.class, BOTTOM_MOTOR_NAME);
        indexTopMotor    = hardwareMap.get(DcMotor.class, TOP_MOTOR_NAME);
        distanceSensor   = hardwareMap.get(DistanceSensor.class, DISTANCE_SENSOR_NAME);

        indexBottomMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexBottomMotor.setDirection(DcMotor.Direction.REVERSE);

        indexTopMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        indexTopMotor.setDirection(DcMotor.Direction.REVERSE);

        stop();
    }

    @Override
    public void periodic() {
        // INTAKING only: stop top motor when artifact is captured; bottom holds it in place
        if (mode == Mode.INTAKING && hasArtifact()) {
            indexTopMotor.setPower(0);
        }
    }

    // ---- Status -------------------------------------------------------

    public Status getStatus() {
        return Status.valueOf(mode.name());
    }

    public boolean isIdle()      { return mode == Mode.IDLE; }
    public boolean isIntaking()  { return mode == Mode.INTAKING; }
    public boolean isFeeding()   { return mode == Mode.FEEDING; }
    public boolean isReversing() { return mode == Mode.REVERSING; }

    public boolean hasArtifact() {
        return distanceSensor.getDistance(DistanceUnit.CM) < ARTIFACT_DISTANCE_THRESHOLD_CM;
    }

    // ---- Commands -----------------------------------------------------

    public void intake() {
        mode = Mode.INTAKING;
        indexBottomMotor.setPower(FORWARD_SPEED);
        indexTopMotor.setPower(FORWARD_SPEED);
    }

    public void feed() {
        mode = Mode.FEEDING;
        indexBottomMotor.setPower(FORWARD_SPEED);
        indexTopMotor.setPower(FORWARD_SPEED);
    }

    public void stop() {
        mode = Mode.IDLE;
        indexBottomMotor.setPower(0);
        indexTopMotor.setPower(0);
    }

    public void reverse() {
        mode = Mode.REVERSING;
        indexBottomMotor.setPower(REVERSE_SPEED);
        indexTopMotor.setPower(REVERSE_SPEED);
    }
}
