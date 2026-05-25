package org.firstinspires.ftc.teamcode.common.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.common.config.RobotConfig;
import java.util.function.DoubleSupplier;

public class MecanumDriveSubsystem extends SubsystemBase {

    // Hardware config names
    private static final String LEFT_FRONT_NAME  = "leftFront";
    private static final String LEFT_BACK_NAME   = "leftBack";
    private static final String RIGHT_FRONT_NAME = "rightFront";
    private static final String RIGHT_BACK_NAME  = "rightBack";

    // PRECISION speed-mode multiplier (applied on top of baseSpeed)
    private static final double PRECISION_MULTIPLIER = 0.5;

    public enum HeadingMode { FIELD_CENTRIC, ROBOT_CENTRIC }
    public enum SpeedMode   { NORMAL, PRECISION, TURBO }

    private final DcMotorEx leftFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightBack;

    private final DoubleSupplier headingSupplier;

    private HeadingMode headingMode = HeadingMode.FIELD_CENTRIC;
    private SpeedMode   speedMode   = SpeedMode.NORMAL;

    private double baseSpeed;
    private double sensitivity;
    private double deadzone;

    private double headingOffset  = 0.0;

    private double storedForward  = 0.0;
    private double storedStrafe   = 0.0;
    private double storedRotate   = 0.0;

    /**
     * @param headingSupplier Returns the robot heading in <b>radians</b>. Any
     *                        degrees-to-radians conversion for Pinpoint or IMU sources
     *                        must happen at the wiring layer (TeleOp constructor), not here.
     */
    public MecanumDriveSubsystem(HardwareMap hardwareMap,
                                 DoubleSupplier headingSupplier,
                                 RobotConfig config) {
        leftFront  = hardwareMap.get(DcMotorEx.class, LEFT_FRONT_NAME);
        leftBack   = hardwareMap.get(DcMotorEx.class, LEFT_BACK_NAME);
        rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT_NAME);
        rightBack  = hardwareMap.get(DcMotorEx.class, RIGHT_BACK_NAME);

        // Standard goBilda mecanum convention (rightFront + rightBack reversed per inventory)
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.headingSupplier = headingSupplier;
        this.baseSpeed       = config.defaultBaseSpeed();
        this.sensitivity     = config.defaultSensitivity();
        this.deadzone        = config.defaultDeadzone();
    }

    @Override
    public void periodic() {
        // 1. Deadzone
        double y = applyDeadzone(storedForward);
        double x = applyDeadzone(storedStrafe);
        double r = applyDeadzone(storedRotate);

        // 2. Sensitivity curve: sign * |v|^sensitivity
        y = applySensitivity(y);
        x = applySensitivity(x);
        r = applySensitivity(r);

        // 3. Speed-mode multiplier (Option B: TURBO bypasses baseSpeed entirely)
        if (speedMode == SpeedMode.TURBO) {
            // no scaling — full power available
        } else if (speedMode == SpeedMode.PRECISION) {
            double scale = baseSpeed * PRECISION_MULTIPLIER;
            y *= scale;
            x *= scale;
            r *= scale;
        } else {
            y *= baseSpeed;
            x *= baseSpeed;
            r *= baseSpeed;
        }

        // 4. Field-centric rotation
        if (headingMode == HeadingMode.FIELD_CENTRIC) {
            double heading  = getHeading();
            double rotatedY =  y * Math.cos(-heading) - x * Math.sin(-heading);
            double rotatedX =  y * Math.sin(-heading) + x * Math.cos(-heading);
            y = rotatedY;
            x = rotatedX;
        }

        // 5. Mecanum mixing (standard sign convention)
        double lf = y + x + r;
        double lb = y - x + r;
        double rf = y - x - r;
        double rb = y + x - r;

        // 6. Normalize
        double max = Math.max(
                Math.max(Math.abs(lf), Math.abs(lb)),
                Math.max(Math.abs(rf), Math.abs(rb)));
        if (max > 1.0) {
            lf /= max;
            lb /= max;
            rf /= max;
            rb /= max;
        }

        // 7. Set motor powers
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }

    // ---- Verb methods (inputs) ----------------------------------------

    /** Stores the requested drive velocities; actual motor output is applied in periodic(). */
    public void drive(double forward, double strafe, double rotate) {
        storedForward = forward;
        storedStrafe  = strafe;
        storedRotate  = rotate;
    }

    /** Zeroes all stored velocities; motors coast to stop on the next periodic() tick. */
    public void stop() {
        storedForward = 0.0;
        storedStrafe  = 0.0;
        storedRotate  = 0.0;
    }

    public void setHeadingMode(HeadingMode mode) {
        headingMode = mode;
    }

    public void toggleHeadingMode() {
        headingMode = (headingMode == HeadingMode.FIELD_CENTRIC)
                ? HeadingMode.ROBOT_CENTRIC
                : HeadingMode.FIELD_CENTRIC;
    }

    public void setSpeedMode(SpeedMode mode) {
        speedMode = mode;
    }

    public void setBaseSpeed(double speed) {
        baseSpeed = speed;
    }

    public void setSensitivity(double sensitivity) {
        this.sensitivity = sensitivity;
    }

    public void setDeadzone(double deadzone) {
        this.deadzone = deadzone;
    }

    /**
     * Captures the current heading from the supplier as a zero-point offset.
     * getHeading() returns zero immediately after this call. Does not modify
     * the heading supplier itself.
     */
    public void resetHeading() {
        headingOffset = headingSupplier.getAsDouble();
    }

    // ---- Getters / predicates (outputs) -------------------------------

    public HeadingMode getHeadingMode() { return headingMode; }
    public SpeedMode   getSpeedMode()   { return speedMode; }
    public double      getBaseSpeed()   { return baseSpeed; }
    public double      getSensitivity() { return sensitivity; }
    public double      getDeadzone()    { return deadzone; }

    /** Returns the heading in radians relative to the last resetHeading() call. */
    public double getHeading() {
        return headingSupplier.getAsDouble() - headingOffset;
    }

    /** True when all stored velocities are zero (not derived from motor state). */
    public boolean isStopped() {
        return storedForward == 0.0 && storedStrafe == 0.0 && storedRotate == 0.0;
    }

    public boolean isFieldCentric() {
        return headingMode == HeadingMode.FIELD_CENTRIC;
    }

    // ---- Helpers ------------------------------------------------------

    private double applyDeadzone(double value) {
        return Math.abs(value) > deadzone ? value : 0.0;
    }

    private double applySensitivity(double value) {
        if (value == 0.0) return 0.0;
        return Math.copySign(Math.pow(Math.abs(value), sensitivity), value);
    }
}
