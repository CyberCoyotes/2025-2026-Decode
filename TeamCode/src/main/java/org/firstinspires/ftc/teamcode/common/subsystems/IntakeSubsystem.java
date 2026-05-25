package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem extends SubsystemBase {

    // Hardware config names
    private static final String WHEEL_LEFT_NAME  = "leftIntake";
    private static final String WHEEL_RIGHT_NAME = "rightIntake";
    private static final String SLIDE_LEFT_NAME  = "leftSlide";
    private static final String SLIDE_RIGHT_NAME = "rightSlide";

    // Wheel servo speeds (continuous rotation; position = STOP + speed * 0.5)
    private static final double STOP_POSITION = 0.5;
    private static final double INTAKE_SPEED  = -1.0;
    private static final double EJECT_SPEED   =  1.0;

    // Slide servo positions (position servos)
    private static final double SLIDE_IN_POSITION  = 0.0;
    private static final double SLIDE_OUT_POSITION = 0.85; // 0.90 caused overheating at full extension

    // FINISHING window: slides retract immediately; wheels keep running until this elapses
    private static final long WHEEL_STOP_DELAY_MS = 600;

    public enum Status {
        IDLE,
        INTAKING,
        EJECTING,
        FINISHING,
        MANUAL
    }

    private enum Direction { NONE, FORWARD, REVERSE }

    private final Servo intakeWheelLeft;
    private final Servo intakeWheelRight;
    private final Servo intakeSlideLeft;
    private final Servo intakeSlideRight;

    private Direction wheelDirection = Direction.NONE;
    private long      stopTimestamp  = 0;
    private boolean   manualOverride = false;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeWheelLeft  = hardwareMap.get(Servo.class, WHEEL_LEFT_NAME);
        intakeWheelRight = hardwareMap.get(Servo.class, WHEEL_RIGHT_NAME);
        intakeSlideLeft  = hardwareMap.get(Servo.class, SLIDE_LEFT_NAME);
        intakeSlideRight = hardwareMap.get(Servo.class, SLIDE_RIGHT_NAME);

        intakeWheelLeft.setDirection(Servo.Direction.FORWARD);
        intakeWheelRight.setDirection(Servo.Direction.REVERSE);
        intakeSlideLeft.setDirection(Servo.Direction.FORWARD);
        intakeSlideRight.setDirection(Servo.Direction.REVERSE);

        setWheelHardware(Direction.NONE);
        setSlidesHardware(false);
    }

    @Override
    public void periodic() {
        if (stopTimestamp != 0 &&
                System.currentTimeMillis() - stopTimestamp >= WHEEL_STOP_DELAY_MS) {
            wheelDirection = Direction.NONE;
            setWheelHardware(Direction.NONE);
            stopTimestamp = 0;
        }
    }

    // ---- Status -------------------------------------------------------

    public Status getStatus() {
        if (manualOverride)                      return Status.MANUAL;
        if (stopTimestamp != 0)                  return Status.FINISHING;
        if (wheelDirection == Direction.FORWARD) return Status.INTAKING;
        if (wheelDirection == Direction.REVERSE) return Status.EJECTING;
        return Status.IDLE;
    }

    public boolean isIdle()             { return getStatus() == Status.IDLE; }
    public boolean isInManualOverride() { return manualOverride; }
    public boolean isExtended()         { return intakeSlideLeft.getPosition() == SLIDE_OUT_POSITION; }

    public boolean isRunning() {
        Status s = getStatus();
        return s == Status.INTAKING || s == Status.EJECTING || s == Status.FINISHING;
    }

    // ---- Commands -----------------------------------------------------

    public void intake() {
        manualOverride = false;
        stopTimestamp  = 0;
        wheelDirection = Direction.FORWARD;
        setWheelHardware(Direction.FORWARD);
        setSlidesHardware(true);
    }

    public void eject() {
        manualOverride = false;
        stopTimestamp  = 0;
        wheelDirection = Direction.REVERSE;
        setWheelHardware(Direction.REVERSE);
        setSlidesHardware(true);
    }

    public void stop() {
        if (stopTimestamp != 0) return;
        manualOverride = false;
        setSlidesHardware(false);
        if (wheelDirection != Direction.NONE) {
            stopTimestamp = System.currentTimeMillis();
        }
    }

    public void enableManualOverride()  { manualOverride = true; }
    public void disableManualOverride() { manualOverride = false; }

    public void extendSlides() {
        if (!manualOverride) return;
        setSlidesHardware(true);
    }

    public void retractSlides() {
        if (!manualOverride) return;
        setSlidesHardware(false);
    }

    // ---- Hardware helpers ---------------------------------------------

    private void setWheelHardware(Direction dir) {
        double speed;
        switch (dir) {
            case FORWARD: speed = INTAKE_SPEED; break;
            case REVERSE: speed = EJECT_SPEED;  break;
            default:      speed = 0.0;          break;
        }
        double position = STOP_POSITION + (speed * 0.5);
        intakeWheelLeft.setPosition(position);
        intakeWheelRight.setPosition(position);
    }

    private void setSlidesHardware(boolean extended) {
        double position = extended ? SLIDE_OUT_POSITION : SLIDE_IN_POSITION;
        intakeSlideLeft.setPosition(position);
        intakeSlideRight.setPosition(position);
    }
}
