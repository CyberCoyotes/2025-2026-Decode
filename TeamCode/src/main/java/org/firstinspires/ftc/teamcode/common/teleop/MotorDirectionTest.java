package org.firstinspires.ftc.teamcode.common.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Motor Direction Test OpMode
 *
 * This OpMode helps verify that all motors are wired correctly and spinning
 * in the correct direction for mecanum drive.
 *
 * CONTROLS:
 * Gamepad 1:
 *   Y button - Test LEFT FRONT motor forward
 *   B button - Test RIGHT FRONT motor forward
 *   A button - Test RIGHT BACK motor forward
 *   X button - Test LEFT BACK motor forward
 *
 *   D-pad Up    - All motors forward (robot should move forward)
 *   D-pad Down  - All motors backward (robot should move backward)
 *   D-pad Left  - Strafe left pattern
 *   D-pad Right - Strafe right pattern
 *
 *   Left Bumper  - Rotate CCW (counter-clockwise)
 *   Right Bumper - Rotate CW (clockwise)
 *
 * EXPECTED BEHAVIOR:
 * - Y: LEFT FRONT wheel spins to push robot FORWARD
 * - B: RIGHT FRONT wheel spins to push robot FORWARD
 * - A: RIGHT BACK wheel spins to push robot FORWARD
 * - X: LEFT BACK wheel spins to push robot FORWARD
 *
 * For mecanum wheels:
 * - Front left and back right wheels have rollers angled one way (\)
 * - Front right and back left wheels have rollers angled opposite way (/)
 *
 * When all move forward at same speed, robot should drive straight forward.
 */
@Disabled
@TeleOp(name = "Motor Direction Test", group = "ALPHA")
public class MotorDirectionTest extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private static final double TEST_POWER = 0.5;  // Half speed for testing

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Set to brake mode for better control
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Current motor directions from MecanumDriveSubsystem
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("=== MOTOR DIRECTION TEST ===");
        telemetry.addLine();
        telemetry.addLine("INDIVIDUAL MOTOR TEST:");
        telemetry.addLine("Y - Test Left Front");
        telemetry.addLine("B - Test Right Front");
        telemetry.addLine("A - Test Right Back");
        telemetry.addLine("X - Test Left Back");
        telemetry.addLine();
        telemetry.addLine("PATTERN TESTS:");
        telemetry.addLine("D-pad Up    - Forward");
        telemetry.addLine("D-pad Down  - Backward");
        telemetry.addLine("D-pad Left  - Strafe Left");
        telemetry.addLine("D-pad Right - Strafe Right");
        telemetry.addLine();
        telemetry.addLine("ROTATION TESTS:");
        telemetry.addLine("Left Bumper  - Rotate CCW");
        telemetry.addLine("Right Bumper - Rotate CW");
        telemetry.addLine();
        telemetry.addData("Status", "Ready - Press START");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Stop all motors by default
            double lfPower = 0;
            double lbPower = 0;
            double rfPower = 0;
            double rbPower = 0;

            String activeTest = "NONE - Press a button";

            // Individual motor tests
            if (gamepad1.y) {
                lfPower = TEST_POWER;
                activeTest = "LEFT FRONT FORWARD";
            } else if (gamepad1.b) {
                rfPower = TEST_POWER;
                activeTest = "RIGHT FRONT FORWARD";
            } else if (gamepad1.a) {
                rbPower = TEST_POWER;
                activeTest = "RIGHT BACK FORWARD";
            } else if (gamepad1.x) {
                lbPower = TEST_POWER;
                activeTest = "LEFT BACK FORWARD";
            }

            // Pattern tests - all four motors together
            else if (gamepad1.dpad_up) {
                // Forward - all motors same direction
                lfPower = TEST_POWER;
                lbPower = TEST_POWER;
                rfPower = TEST_POWER;
                rbPower = TEST_POWER;
                activeTest = "ALL FORWARD - Robot should move FORWARD";
            } else if (gamepad1.dpad_down) {
                // Backward - all motors reversed
                lfPower = -TEST_POWER;
                lbPower = -TEST_POWER;
                rfPower = -TEST_POWER;
                rbPower = -TEST_POWER;
                activeTest = "ALL BACKWARD - Robot should move BACKWARD";
            } else if (gamepad1.dpad_left) {
                // Strafe left pattern for mecanum
                lfPower = -TEST_POWER;
                lbPower = TEST_POWER;
                rfPower = TEST_POWER;
                rbPower = -TEST_POWER;
                activeTest = "STRAFE LEFT - Robot should move LEFT";
            } else if (gamepad1.dpad_right) {
                // Strafe right pattern for mecanum
                lfPower = TEST_POWER;
                lbPower = -TEST_POWER;
                rfPower = -TEST_POWER;
                rbPower = TEST_POWER;
                activeTest = "STRAFE RIGHT - Robot should move RIGHT";
            }

            // Rotation tests
            else if (gamepad1.left_bumper) {
                // Rotate counter-clockwise
                lfPower = -TEST_POWER;
                lbPower = -TEST_POWER;
                rfPower = TEST_POWER;
                rbPower = TEST_POWER;
                activeTest = "ROTATE CCW - Robot should spin LEFT";
            } else if (gamepad1.right_bumper) {
                // Rotate clockwise
                lfPower = TEST_POWER;
                lbPower = TEST_POWER;
                rfPower = -TEST_POWER;
                rbPower = -TEST_POWER;
                activeTest = "ROTATE CW - Robot should spin RIGHT";
            }

            // Set motor powers
            leftFront.setPower(lfPower);
            leftBack.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack.setPower(rbPower);

            // Telemetry
            telemetry.clearAll();
            telemetry.addLine("=== MOTOR DIRECTION TEST ===");
            telemetry.addLine();
            telemetry.addData("Active Test", activeTest);
            telemetry.addLine();
            telemetry.addLine("--- MOTOR POWERS ---");
            telemetry.addData("Left Front (Y)", "%.2f", lfPower);
            telemetry.addData("Right Front (B)", "%.2f", rfPower);
            telemetry.addData("Right Back (A)", "%.2f", rbPower);
            telemetry.addData("Left Back (X)", "%.2f", lbPower);
            telemetry.addLine();
            telemetry.addLine("--- CURRENT MOTOR DIRECTIONS ---");
            telemetry.addData("Left Front", leftFront.getDirection());
            telemetry.addData("Left Back", leftBack.getDirection());
            telemetry.addData("Right Front", rightFront.getDirection());
            telemetry.addData("Right Back", rightBack.getDirection());
            telemetry.addLine();
            telemetry.addLine("TROUBLESHOOTING:");
            telemetry.addLine("If motors spin wrong direction:");
            telemetry.addLine("1. Note which motor(s) are backwards");
            telemetry.addLine("2. Update MecanumDriveSubsystem");
            telemetry.addLine("   motor directions (lines 74-77)");
            telemetry.update();
        }

        // Stop all motors when done
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
