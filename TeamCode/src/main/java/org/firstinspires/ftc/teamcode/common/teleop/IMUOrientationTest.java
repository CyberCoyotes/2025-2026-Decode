package org.firstinspires.ftc.teamcode.common.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * IMU Orientation Test OpMode
 *
 * This OpMode helps verify that the IMU is configured correctly for field-centric driving.
 *
 * CONTROLS:
 * Gamepad 1:
 *   Y button - Reset IMU heading to 0
 *
 *   D-pad Up/Down    - Cycle through LogoFacingDirection options
 *   D-pad Left/Right - Cycle through UsbFacingDirection options
 *
 * EXPECTED BEHAVIOR with CORRECT orientation:
 * - Yaw should be near 0° when robot faces forward
 * - Yaw should INCREASE when robot rotates COUNTER-CLOCKWISE
 * - Yaw should DECREASE when robot rotates CLOCKWISE
 * - Pitch should INCREASE when front of robot tips UP
 * - Roll should INCREASE when left side of robot tips UP
 *
 * Use this to find the correct orientation settings for your hub mounting.
 */
@Disabled
@TeleOp(name = "IMU Orientation Test", group = "ALPHA")
public class IMUOrientationTest extends LinearOpMode {

    private IMU imu;

    // All possible orientations
    private RevHubOrientationOnRobot.LogoFacingDirection[] logoDirections = {
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT
    };

    private RevHubOrientationOnRobot.UsbFacingDirection[] usbDirections = {
        RevHubOrientationOnRobot.UsbFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.DOWN,
        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD,
        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD,
        RevHubOrientationOnRobot.UsbFacingDirection.LEFT,
        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
    };

    private int logoIndex = 2;  // Start with FORWARD
    private int usbIndex = 0;   // Start with UP

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastY = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Start with the current competition configuration
        updateIMUOrientation();

        telemetry.addLine("=== IMU ORIENTATION TEST ===");
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("Y - Reset heading to 0°");
        telemetry.addLine("D-pad Up/Down - Change Logo direction");
        telemetry.addLine("D-pad Left/Right - Change USB direction");
        telemetry.addLine();
        telemetry.addLine("TEST THE FOLLOWING:");
        telemetry.addLine("1. Reset heading (Y button)");
        telemetry.addLine("2. Rotate robot CCW - Yaw should INCREASE");
        telemetry.addLine("3. Rotate robot CW - Yaw should DECREASE");
        telemetry.addLine();
        telemetry.addData("Status", "Ready - Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Handle reset heading
            if (gamepad1.y && !lastY) {
                imu.resetYaw();
                gamepad1.rumble(300);
            }
            lastY = gamepad1.y;

            // Handle logo direction change
            if (gamepad1.dpad_up && !lastDpadUp) {
                logoIndex = (logoIndex + 1) % logoDirections.length;
                updateIMUOrientation();
                gamepad1.rumble(100);
            }
            lastDpadUp = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !lastDpadDown) {
                logoIndex = (logoIndex - 1 + logoDirections.length) % logoDirections.length;
                updateIMUOrientation();
                gamepad1.rumble(100);
            }
            lastDpadDown = gamepad1.dpad_down;

            // Handle USB direction change
            if (gamepad1.dpad_right && !lastDpadRight) {
                usbIndex = (usbIndex + 1) % usbDirections.length;
                updateIMUOrientation();
                gamepad1.rumble(100);
            }
            lastDpadRight = gamepad1.dpad_right;

            if (gamepad1.dpad_left && !lastDpadLeft) {
                usbIndex = (usbIndex - 1 + usbDirections.length) % usbDirections.length;
                updateIMUOrientation();
                gamepad1.rumble(100);
            }
            lastDpadLeft = gamepad1.dpad_left;

            // Get IMU data
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            double yawDeg = angles.getYaw(AngleUnit.DEGREES);
            double pitchDeg = angles.getPitch(AngleUnit.DEGREES);
            double rollDeg = angles.getRoll(AngleUnit.DEGREES);
            double yawRad = angles.getYaw(AngleUnit.RADIANS);

            // Telemetry
            telemetry.clearAll();
            telemetry.addLine("=== IMU ORIENTATION TEST ===");
            telemetry.addLine();

            telemetry.addLine("--- CURRENT CONFIGURATION ---");
            telemetry.addData("Logo Facing", logoDirections[logoIndex]);
            telemetry.addData("USB Facing", usbDirections[usbIndex]);
            telemetry.addLine();

            telemetry.addLine("--- IMU READINGS ---");
            telemetry.addData("Yaw (Heading)", "%.1f°", yawDeg);
            telemetry.addData("Pitch", "%.1f°", pitchDeg);
            telemetry.addData("Roll", "%.1f°", rollDeg);
            telemetry.addData("Yaw Radians", "%.3f", yawRad);
            telemetry.addLine();

            telemetry.addLine("--- VERIFICATION CHECKLIST ---");
            telemetry.addLine("✓ Yaw INCREASES when rotate CCW?");
            telemetry.addLine("✓ Yaw DECREASES when rotate CW?");
            telemetry.addLine("✓ Pitch INCREASES when front tips UP?");
            telemetry.addLine("✓ Roll INCREASES when left tips UP?");
            telemetry.addLine();

            telemetry.addLine("--- CONTROLS ---");
            telemetry.addLine("Y: Reset Heading");
            telemetry.addLine("D-pad Up/Down: Change Logo direction");
            telemetry.addLine("D-pad Left/Right: Change USB direction");

            telemetry.update();
        }
    }

    private void updateIMUOrientation() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoDirections[logoIndex];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbDirections[usbIndex];

        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
        } catch (IllegalArgumentException e) {
            // Invalid combination - revert
            telemetry.addData("Error", "Invalid orientation combination");
        }
    }
}
