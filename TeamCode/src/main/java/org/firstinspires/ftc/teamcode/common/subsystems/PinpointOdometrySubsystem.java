package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

/**
 * GoBilda Pinpoint Odometry Computer Subsystem for Teams 11940 & 22091
 *
 * Provides accurate position tracking and heading information using:
 * - Two perpendicular dead wheel odometry pods
 * - Integrated IMU (LSM6DSV16X)
 * - Pose Exponential algorithm for position calculation
 *
 * HARDWARE CONFIGURATION:
 * - Module is mounted UPSIDE DOWN underneath the chassis
 * - Located approximately CENTER of robot front-to-back
 * - Located approximately 3.75 inches RIGHT of center (left-to-right)
 * - Two goBILDA spring-loaded swing-arm odometry pods
 *
 * COORDINATE SYSTEM:
 * - X axis: Forward/backward (forward is positive)
 * - Y axis: Left/right (left is positive)
 * - Heading: Clockwise is positive (inverted due to upside-down IMU mounting)
 *
 * WIRING:
 * - Connect to I2C port on Control Hub or Expansion Hub
 * - Hardware map name: "odo"
 * - X pod (forward): should count up when robot moves forward
 * - Y pod (strafe): should count up when robot moves left
 */
public class PinpointOdometrySubsystem {

    /* ========================================
     * HARDWARE
     * ======================================== */
    private final GoBildaPinpointDriver odo;

    /* ========================================
     * CONFIGURATION CONSTANTS
     * ======================================== */
    // Hardware name in configuration
    private static final String ODOMETRY_DEVICE_NAME = "odo";

    // Pod offsets relative to robot center of rotation (in millimeters)
    // X offset: sideways distance (left positive, right negative)
    // Y offset: forward/back distance (forward positive, back negative)
    //
    // CURRENT CONFIGURATION:
    // - X: -95.25 mm (3.75 inches to the right of center)
    // - Y: 0 mm (centered front-to-back, adjustable if needed)
    private static final double X_OFFSET_MM = -95.25;  // 3.75 inches right of center
    private static final double Y_OFFSET_MM = 0.0;     // Centered front-to-back (configurable)

    // Encoder resolution (goBILDA spring-loaded swing-arm pods)
    private static final GoBildaPinpointDriver.GoBildaOdometryPods ENCODER_RESOLUTION =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;

    // Encoder directions for UPSIDE-DOWN mounting with module FACING FORWARD
    // X pod: FORWARD - X should increase when robot moves forward
    // Y pod: FORWARD - Y should increase when robot moves left
    private static final GoBildaPinpointDriver.EncoderDirection X_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;  // X increases when moving forward
    private static final GoBildaPinpointDriver.EncoderDirection Y_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;  // Y increases when moving left

    // Heading inversion for upside-down IMU mounting
    // When Pinpoint is upside down, the IMU's yaw axis may be inverted
    // Set to true to negate heading values (makes clockwise positive)
    private static final boolean INVERT_HEADING = true;

    // Distance calibration multiplier
    // Used to correct odometry distance measurements based on real-world testing
    // To calibrate: measure actual distance traveled, divide by reported distance
    // Example: Robot travels 9 inches, odometry reports 6 inches → multiplier = 9/6 = 1.5
    //
    // CURRENT CONFIGURATION:
    // - Odometry pods: 48mm diameter wheels (43mm width), 2048 PPR encoder
    // - Swing-arm pods (spring-loaded, single pivot point)
    // - goBILDA_SWINGARM_POD default is configured for different encoder resolution
    // - Measured: 9 inches actual / 6 inches reported = 1.5x multiplier needed
    //
    // NOTE: The 1.5x error suggests encoder resolution mismatch. Standard goBILDA pods
    // may use 8192 CPR (counts per revolution) while 2048 PPR encoders in quadrature
    // mode should also produce 8192 CPR. The discrepancy may require further investigation.
    private static final double DISTANCE_CALIBRATION_MULTIPLIER = 1.5;

    /* ========================================
     * CACHED VALUES
     * ======================================== */
    private Pose2D currentPosition = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    private double loopTime = 0;
    private double frequency = 0;
    private long lastUpdateTime = 0;

    /* ========================================
     * CONSTRUCTOR
     * ======================================== */

    /**
     * Initialize the Pinpoint Odometry Computer
     *
     * @param hardwareMap The hardware map from the OpMode
     */
    public PinpointOdometrySubsystem(HardwareMap hardwareMap) {
        // Initialize the Pinpoint driver
        odo = hardwareMap.get(GoBildaPinpointDriver.class, ODOMETRY_DEVICE_NAME);

        // Configure pod offsets
        // These define how far the odometry pods are from the robot's center of rotation
        odo.setOffsets(X_OFFSET_MM, Y_OFFSET_MM, DistanceUnit.MM);

        // Configure encoder resolution (goBILDA 4-bar pods)
        odo.setEncoderResolution(ENCODER_RESOLUTION);

        // Configure encoder directions for upside-down mounting
        odo.setEncoderDirections(X_ENCODER_DIRECTION, Y_ENCODER_DIRECTION);

        // Reset position and calibrate IMU
        // This should be called when robot is stationary
        odo.resetPosAndIMU();

        lastUpdateTime = System.nanoTime();
    }

    /* ========================================
     * UPDATE METHOD
     * ======================================== */

    /**
     * Update the odometry system
     * Call this method once per loop to get the latest position data
     *
     * IMPORTANT: This must be called regularly (ideally every loop iteration)
     * for accurate position tracking
     */
    public void update() {
        // Request update from Pinpoint (reads all sensors via I2C)
        odo.update();

        // Cache the current position
        Pose2D rawPosition = odo.getPosition();

        // Apply heading inversion if needed for upside-down mounting
        if (INVERT_HEADING) {
            currentPosition = new Pose2D(
                    DistanceUnit.MM,
                    rawPosition.getX(DistanceUnit.MM),
                    rawPosition.getY(DistanceUnit.MM),
                    AngleUnit.DEGREES,
                    -rawPosition.getHeading(AngleUnit.DEGREES)
            );
        } else {
            currentPosition = rawPosition;
        }

        // Calculate loop frequency
        long currentTime = System.nanoTime();
        loopTime = (currentTime - lastUpdateTime) / 1_000_000_000.0;  // Convert to seconds
        frequency = (loopTime > 0) ? 1.0 / loopTime : 0;
        lastUpdateTime = currentTime;
    }

    /**
     * Update only the heading (faster than full update)
     * Use this if you only need heading information and want to save I2C bandwidth
     */
    public void updateHeadingOnly() {
        odo.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        Pose2D rawPosition = odo.getPosition();

        // Apply heading inversion if needed for upside-down mounting
        if (INVERT_HEADING) {
            currentPosition = new Pose2D(
                    DistanceUnit.MM,
                    rawPosition.getX(DistanceUnit.MM),
                    rawPosition.getY(DistanceUnit.MM),
                    AngleUnit.DEGREES,
                    -rawPosition.getHeading(AngleUnit.DEGREES)
            );
        } else {
            currentPosition = rawPosition;
        }

        // Update timing
        long currentTime = System.nanoTime();
        loopTime = (currentTime - lastUpdateTime) / 1_000_000_000.0;
        frequency = (loopTime > 0) ? 1.0 / loopTime : 0;
        lastUpdateTime = currentTime;
    }

    /* ========================================
     * POSITION & HEADING GETTERS
     * ======================================== */

    /**
     * Get the current robot position as a Pose2D
     * Contains X, Y, and heading
     *
     * @return Current position (default units: MM and DEGREES)
     */
    public Pose2D getPosition() {
        return currentPosition;
    }

    /**
     * Get X position (forward/backward)
     * Applies distance calibration multiplier for accurate measurements
     *
     * @param unit Distance unit (MM, CM, INCH, etc.)
     * @return X coordinate (calibrated)
     */
    public double getX(DistanceUnit unit) {
        return currentPosition.getX(unit) * DISTANCE_CALIBRATION_MULTIPLIER;
    }

    /**
     * Get Y position (left/right)
     * Applies distance calibration multiplier for accurate measurements
     *
     * @param unit Distance unit (MM, CM, INCH, etc.)
     * @return Y coordinate (calibrated)
     */
    public double getY(DistanceUnit unit) {
        return currentPosition.getY(unit) * DISTANCE_CALIBRATION_MULTIPLIER;
    }

    /**
     * Get the robot heading (rotation angle)
     *
     * @param unit Angle unit (DEGREES or RADIANS)
     * @return Heading angle
     */
    public double getHeading(AngleUnit unit) {
        return currentPosition.getHeading(unit);
    }

    /**
     * Get the robot heading in degrees
     * Convenience method for field-centric drive
     *
     * @return Heading in degrees
     */
    public double getHeadingDegrees() {
        return currentPosition.getHeading(AngleUnit.DEGREES);
    }

    /**
     * Get the robot heading in radians
     * Convenience method for field-centric drive
     *
     * @return Heading in radians
     */
    public double getHeadingRadians() {
        return currentPosition.getHeading(AngleUnit.RADIANS);
    }

    /* ========================================
     * VELOCITY GETTERS
     * ======================================== */

    /**
     * Get X velocity (forward/backward speed)
     *
     * @param unit Distance unit for velocity
     * @return X velocity in units/second
     */
    public double getVelocityX(DistanceUnit unit) {
        return odo.getVelX(unit);
    }

    /**
     * Get Y velocity (strafe speed)
     *
     * @param unit Distance unit for velocity
     * @return Y velocity in units/second
     */
    public double getVelocityY(DistanceUnit unit) {
        return odo.getVelY(unit);
    }

    /**
     * Get heading velocity (rotation speed)
     *
     * @param unit Angle unit for velocity (DEGREES or RADIANS)
     * @return Heading velocity in units/second
     */
    public double getHeadingVelocity(UnnormalizedAngleUnit unit) {
        double rawVelocity = odo.getHeadingVelocity(unit);
        return INVERT_HEADING ? -rawVelocity : rawVelocity;
    }

    /* ========================================
     * RESET & CALIBRATION METHODS
     * ======================================== */

    /**
     * Reset position to (0, 0, 0) and recalibrate the IMU
     * Robot must be stationary when calling this method
     *
     * Recommended usage:
     * - At the start of autonomous mode
     * - When repositioning robot to a known location
     */
    public void resetPosAndIMU() {
        odo.resetPosAndIMU();
        currentPosition = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0);
    }

    /**
     * Recalibrate the IMU without resetting position
     * Robot must be stationary when calling this method
     *
     * Use this if heading drift is observed but position is still accurate
     */
    public void recalibrateIMU() {
        odo.recalibrateIMU();
    }

    /**
     * Set the robot position to a specific pose
     * Useful for setting a known starting position in autonomous
     *
     * @param pose The new position to set
     */
    public void setPosition(Pose2D pose) {
        odo.setPosition(pose);
        currentPosition = pose;
    }

    /* ========================================
     * DIAGNOSTIC & STATUS METHODS
     * ======================================== */

    /**
     * Get the current device status
     *
     * Possible status values:
     * - READY: Device is working normally
     * - CALIBRATING: IMU is calibrating, outputs are on hold
     * - NOT_READY: Device is resetting (after power cycle)
     * - FAULT_NO_PODS_DETECTED: No odometry pods detected
     * - FAULT_X_POD_NOT_DETECTED: X pod not detected
     * - FAULT_Y_POD_NOT_DETECTED: Y pod not detected
     * - FAULT_BAD_READ: Bad I2C read detected
     *
     * @return Current device status
     */
    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {
        return odo.getDeviceStatus();
    }

    /**
     * Check if the device is ready for use
     *
     * @return true if status is READY, false otherwise
     */
    public boolean isReady() {
        return odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    /**
     * Get the Pinpoint device update frequency
     *
     * @return Frequency in Hz
     */
    public double getPinpointFrequency() {
        return odo.getFrequency();
    }

    /**
     * Get the Control Hub loop frequency
     *
     * @return Frequency in Hz
     */
    public double getLoopFrequency() {
        return frequency;
    }

    /**
     * Get the firmware version of the Pinpoint device
     *
     * @return Version number as string
     */
    public String getDeviceVersion() {
        return String.valueOf(odo.getDeviceVersion());
    }

    /**
     * Get the yaw scalar (heading calibration value)
     * This can be adjusted if heading drift is observed
     *
     * @return Yaw scalar value
     */
    public double getYawScalar() {
        return odo.getYawScalar();
    }

    /* ========================================
     * TELEMETRY METHODS
     * ======================================== */

    /**
     * Get formatted position string for telemetry
     * Format: {X: 123.456, Y: 123.456, H: 123.456}
     *
     * @param distanceUnit Unit for X and Y
     * @param angleUnit Unit for heading
     * @return Formatted position string
     */
    public String getPositionTelemetry(DistanceUnit distanceUnit, AngleUnit angleUnit) {
        return String.format(Locale.US, "{X: %.2f, Y: %.2f, H: %.2f}",
                currentPosition.getX(distanceUnit),
                currentPosition.getY(distanceUnit),
                currentPosition.getHeading(angleUnit));
    }

    /**
     * Get formatted velocity string for telemetry
     * Format: {XVel: 123.456, YVel: 123.456, HVel: 123.456}
     *
     * @param distanceUnit Unit for X and Y velocities
     * @param angleUnit Unit for heading velocity
     * @return Formatted velocity string
     */
    public String getVelocityTelemetry(DistanceUnit distanceUnit, UnnormalizedAngleUnit angleUnit) {
        return String.format(Locale.US, "{XVel: %.2f, YVel: %.2f, HVel: %.2f}",
                odo.getVelX(distanceUnit),
                odo.getVelY(distanceUnit),
                odo.getHeadingVelocity(angleUnit));
    }

    /**
     * Get compact telemetry string for dashboard
     * Includes position in inches and heading in degrees
     *
     * @return Compact telemetry string
     */
    public String getTelemetry() {
        return String.format(Locale.US,
                "Pinpoint Odometry\n" +
                "Position: X=%.2f\" Y=%.2f\" H=%.1f°\n" +
                "Velocity: X=%.1f Y=%.1f H=%.1f°/s\n" +
                "Status: %s | Freq: %.0fHz",
                currentPosition.getX(DistanceUnit.INCH),
                currentPosition.getY(DistanceUnit.INCH),
                currentPosition.getHeading(AngleUnit.DEGREES),
                odo.getVelX(DistanceUnit.INCH),
                odo.getVelY(DistanceUnit.INCH),
                odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES),
                odo.getDeviceStatus(),
                odo.getFrequency());
    }

    /**
     * Get detailed telemetry string with all diagnostic info
     *
     * @return Detailed telemetry string
     */
    public String getDetailedTelemetry() {
        return String.format(Locale.US,
                "=== Pinpoint Odometry ===\n" +
                "Position (inches):\n" +
                "  X: %.3f  Y: %.3f  Heading: %.2f°\n" +
                "Velocity (in/s, °/s):\n" +
                "  X: %.2f  Y: %.2f  Heading: %.2f\n" +
                "Configuration:\n" +
                "  X Offset: %.2fmm  Y Offset: %.2fmm\n" +
                "  X Dir: %s  Y Dir: %s\n" +
                "Diagnostics:\n" +
                "  Status: %s\n" +
                "  Device Version: %s\n" +
                "  Pinpoint Freq: %.1fHz  Loop Freq: %.1fHz\n" +
                "  Yaw Scalar: %.4f",
                currentPosition.getX(DistanceUnit.INCH),
                currentPosition.getY(DistanceUnit.INCH),
                currentPosition.getHeading(AngleUnit.DEGREES),
                odo.getVelX(DistanceUnit.INCH),
                odo.getVelY(DistanceUnit.INCH),
                odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES),
                X_OFFSET_MM, Y_OFFSET_MM,
                X_ENCODER_DIRECTION, Y_ENCODER_DIRECTION,
                odo.getDeviceStatus(),
                getDeviceVersion(),
                odo.getFrequency(),
                frequency,
                odo.getYawScalar());
    }

    /* ========================================
     * PEDRO PATHING INTEGRATION
     * ======================================== */

    /**
     * Get position in format compatible with Pedro Pathing 2.0
     * Pedro Pathing uses inches for distance
     *
     * @return Position with X, Y in inches and heading in radians
     */
    public Pose2D getPositionForPedroPathing() {
        return new Pose2D(
                DistanceUnit.INCH,
                currentPosition.getX(DistanceUnit.INCH),
                currentPosition.getY(DistanceUnit.INCH),
                AngleUnit.RADIANS,
                currentPosition.getHeading(AngleUnit.RADIANS)
        );
    }
}
