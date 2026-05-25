package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSubsystem extends SubsystemBase {

    // Hardware config names
    private static final String FLYWHEEL_MOTOR_NAME = "flywheelMotor";
    private static final String HOOD_SERVO_NAME     = "hoodServo";

    // Hood servo limits
    private static final double HOOD_MIN_POSITION     = 0.0;
    private static final double HOOD_MAX_POSITION     = 0.60;
    private static final double HOOD_DEFAULT_POSITION = 0.0;

    // Flywheel motor — goBilda 5203 Yellow Jacket 6000 RPM 1:1
    private static final double FLYWHEEL_CPR = 28.0;

    // Velocity PIDF (tuned; F=22 avoids the overshoot seen at F=24+)
    private static final double FLYWHEEL_P = 5.0;
    private static final double FLYWHEEL_I = 0.1;
    private static final double FLYWHEEL_D = 1.0;
    private static final double FLYWHEEL_F = 22.0;

    // Manual fine-tune increments
    private static final int    RPM_INCREMENT  = 100;
    private static final double HOOD_INCREMENT = 0.05;

    // Reverse speed for jam clearing
    private static final double REVERSE_RPM = -1000.0;

    public enum ShotPreset {
        LONG_RANGE  (2800, 0.60),
        MEDIUM_RANGE(2500, 0.60),
        SHORT_RANGE (2200, 0.30);

        private final int    targetRPM;
        private final double hoodPosition;

        ShotPreset(int targetRPM, double hoodPosition) {
            this.targetRPM    = targetRPM;
            this.hoodPosition = hoodPosition;
        }

        public int    getTargetRPM()    { return targetRPM; }
        public double getHoodPosition() { return hoodPosition; }

        public double getVelocity() {
            return (targetRPM / 60.0) * 28.0;
        }
    }

    public enum Status {
        IDLE,
        SPINNING_UP,
        AT_SPEED,
        REVERSING
    }

    private final DcMotorEx flywheelMotor;
    private final Servo     hoodServo;

    private double    targetVelocity = 0.0;
    private ShotPreset currentPreset = ShotPreset.SHORT_RANGE;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);
        hoodServo     = hardwareMap.get(Servo.class,     HOOD_SERVO_NAME);

        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setVelocityPIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);

        hoodServo.setPosition(HOOD_DEFAULT_POSITION);
        flywheelMotor.setVelocity(0);
    }

    @Override
    public void periodic() {
    }

    // ---- Status -------------------------------------------------------

    public Status getStatus() {
        if (targetVelocity < 0) return Status.REVERSING;
        if (targetVelocity == 0) return Status.IDLE;
        double tolerance = targetVelocity * 0.05;
        double actual    = flywheelMotor.getVelocity();
        if (Math.abs(actual - targetVelocity) <= tolerance) return Status.AT_SPEED;
        return Status.SPINNING_UP;
    }

    public boolean isIdle()      { return getStatus() == Status.IDLE; }
    public boolean isReady()     { return getStatus() == Status.AT_SPEED; }
    public boolean isReversing() { return getStatus() == Status.REVERSING; }

    // ---- Commands -----------------------------------------------------

    public void setPreset(ShotPreset preset) {
        currentPreset = preset;
        setTargetRPM(preset.getTargetRPM());
        setHoodPosition(preset.getHoodPosition());
    }

    public void setTargetRPM(double rpm) {
        targetVelocity = (rpm / 60.0) * FLYWHEEL_CPR;
        flywheelMotor.setVelocity(targetVelocity);
    }

    public void setHoodPosition(double position) {
        hoodServo.setPosition(clamp(position, HOOD_MIN_POSITION, HOOD_MAX_POSITION));
    }

    public void stopFlywheel() {
        targetVelocity = 0.0;
        flywheelMotor.setVelocity(0);
    }

    public void reverseFlywheel() {
        targetVelocity = (REVERSE_RPM / 60.0) * FLYWHEEL_CPR;
        flywheelMotor.setVelocity(targetVelocity);
    }

    // ---- Manual fine-tune ---------------------------------------------

    public void incrementFlywheelRPM() {
        setTargetRPM(getTargetRPM() + RPM_INCREMENT);
    }

    public void decrementFlywheelRPM() {
        setTargetRPM(getTargetRPM() - RPM_INCREMENT);
    }

    // ---- Getters ------------------------------------------------------

    public double     getCurrentRPM()   { return (flywheelMotor.getVelocity() / FLYWHEEL_CPR) * 60.0; }
    public double     getTargetRPM()    { return (targetVelocity / FLYWHEEL_CPR) * 60.0; }
    public double     getHoodPosition() { return hoodServo.getPosition(); }
    public double     getFlywheelPower(){ return flywheelMotor.getPower(); }
    public ShotPreset getPreset()       { return currentPreset; }

    // ---- Helper -------------------------------------------------------

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
