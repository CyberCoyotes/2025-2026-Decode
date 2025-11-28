package org.firstinspires.ftc.teamcode.common.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Flywheel RPM Test OpMode
 *
 * Purpose: Diagnose why the GoBILDA 5203 Yellow Jacket motor (6000 RPM rated)
 * is not reaching higher RPM values (3000+, target 5000+).
 *
 * Tests three preset RPM values: 3000, 4000, and 5000 RPM
 * Hood is set to maximum position (0.6) for all tests
 *
 * CONTROLS:
 * - DPAD UP: Increase to next RPM preset (3000 -> 4000 -> 5000)
 * - DPAD DOWN: Decrease to previous RPM preset (5000 -> 4000 -> 3000)
 * - X: Stop flywheel
 * - A: Run at current preset RPM
 *
 * TELEMETRY:
 * - Target vs Actual RPM
 * - Target vs Actual Velocity (ticks/sec)
 * - Motor power output
 * - Error percentage
 * - Battery voltage (affects performance)
 * - PIDF coefficients
 */
@TeleOp(name = "Flywheel RPM Test", group = "Test")
public class FlywheelRPMTest extends LinearOpMode {

    // Hardware
    private DcMotorEx flywheelMotor;
    private Servo hoodServo;

    // Hardware names - must match your robot configuration
    private static final String FLYWHEEL_MOTOR_NAME = "flywheelMotor";
    private static final String HOOD_SERVO_NAME = "hoodServo";

    // Motor specifications
    private static final double FLYWHEEL_CPR = 28.0; // Counts per revolution

    // Test RPM presets
    private static final int[] RPM_PRESETS = {3000, 4000, 5000};
    private int currentPresetIndex = 0;

    // Hood position
    private static final double HOOD_MAX_POSITION = 0.6;

    // PIDF coefficients - these may need tuning for higher speeds
    private static final double FLYWHEEL_P = 5.0;
    private static final double FLYWHEEL_I = 0.1;
    private static final double FLYWHEEL_D = 1.0;
    private static final double FLYWHEEL_F = 20.0;

    // State tracking
    private boolean isRunning = false;
    private boolean lastDpadUpState = false;
    private boolean lastDpadDownState = false;
    private boolean lastAState = false;
    private boolean lastXState = false;

    @Override
    public void runOpMode() throws InterruptedException {

        /* ========================================
         * INITIALIZATION
         * ======================================== */
        telemetry.addData("Status", "Initializing flywheel test...");
        telemetry.update();

        // Initialize motor
        flywheelMotor = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);
        hoodServo = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);

        // Configure flywheel motor for velocity control
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set PIDF coefficients
        flywheelMotor.setVelocityPIDFCoefficients(FLYWHEEL_P, FLYWHEEL_I, FLYWHEEL_D, FLYWHEEL_F);

        // Set hood to max position
        hoodServo.setPosition(HOOD_MAX_POSITION);

        // Display initialization info
        telemetry.clearAll();
        telemetry.addLine("=== FLYWHEEL RPM TEST ===");
        telemetry.addLine();
        telemetry.addLine("Motor: GoBILDA 5203 Yellow Jacket");
        telemetry.addLine("Rated: 6000 RPM free speed");
        telemetry.addLine("CPR: 28 encoder ticks");
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("A: Run at current preset");
        telemetry.addLine("X: Stop flywheel");
        telemetry.addLine("DPAD UP: Next RPM preset");
        telemetry.addLine("DPAD DOWN: Previous RPM preset");
        telemetry.addLine();
        telemetry.addData("Hood Position", "%.2f (MAX)", HOOD_MAX_POSITION);
        telemetry.addLine();
        telemetry.addLine("=== PIDF Coefficients ===");
        telemetry.addData("P", FLYWHEEL_P);
        telemetry.addData("I", FLYWHEEL_I);
        telemetry.addData("D", FLYWHEEL_D);
        telemetry.addData("F", FLYWHEEL_F);
        telemetry.addLine();
        telemetry.addData("Status", "Ready to start!");
        telemetry.update();

        waitForStart();

        /* ========================================
         * MAIN TEST LOOP
         * ======================================== */
        while (opModeIsActive()) {

            /* ========================================
             * BUTTON CONTROLS
             * ======================================== */

            // DPAD UP - Next preset
            if (gamepad1.dpad_up && !lastDpadUpState) {
                currentPresetIndex++;
                if (currentPresetIndex >= RPM_PRESETS.length) {
                    currentPresetIndex = RPM_PRESETS.length - 1;
                }
                gamepad1.rumble(100); // Feedback
            }
            lastDpadUpState = gamepad1.dpad_up;

            // DPAD DOWN - Previous preset
            if (gamepad1.dpad_down && !lastDpadDownState) {
                currentPresetIndex--;
                if (currentPresetIndex < 0) {
                    currentPresetIndex = 0;
                }
                gamepad1.rumble(100); // Feedback
            }
            lastDpadDownState = gamepad1.dpad_down;

            // A button - Run at current preset
            if (gamepad1.a && !lastAState) {
                isRunning = true;
                int targetRPM = RPM_PRESETS[currentPresetIndex];
                double targetVelocity = rpmToVelocity(targetRPM);
                flywheelMotor.setVelocity(targetVelocity);
                gamepad1.rumble(200); // Feedback
            }
            lastAState = gamepad1.a;

            // X button - Stop
            if (gamepad1.x && !lastXState) {
                isRunning = false;
                flywheelMotor.setVelocity(0);
                gamepad1.rumble(500); // Longer rumble for stop
            }
            lastXState = gamepad1.x;

            /* ========================================
             * TELEMETRY - DETAILED DIAGNOSTICS
             * ======================================== */
            telemetry.clearAll();

            int targetRPM = RPM_PRESETS[currentPresetIndex];
            double targetVelocity = rpmToVelocity(targetRPM);
            double actualVelocity = flywheelMotor.getVelocity();
            double actualRPM = velocityToRPM(actualVelocity);
            double motorPower = flywheelMotor.getPower();

            // Calculate error percentage
            double rpmError = 0;
            double errorPercent = 0;
            if (isRunning) {
                rpmError = targetRPM - actualRPM;
                errorPercent = (rpmError / targetRPM) * 100;
            }

            // Header
            telemetry.addLine("=== FLYWHEEL RPM TEST ===");
            telemetry.addLine();

            // Status
            telemetry.addData("Status", isRunning ? "RUNNING" : "STOPPED");
            telemetry.addData("Current Preset", "%d RPM (Preset %d of %d)",
                targetRPM, currentPresetIndex + 1, RPM_PRESETS.length);
            telemetry.addLine();

            // RPM Comparison
            telemetry.addLine("=== RPM ===");
            telemetry.addData("Target RPM", "%d", targetRPM);
            telemetry.addData("Actual RPM", "%.1f", actualRPM);
            telemetry.addData("Error", "%.1f RPM (%.1f%%)", rpmError, errorPercent);

            // Visual indicator of how close we are
            if (isRunning) {
                if (Math.abs(errorPercent) < 5) {
                    telemetry.addData("Performance", "✓ GOOD (within 5%%)");
                } else if (Math.abs(errorPercent) < 15) {
                    telemetry.addData("Performance", "⚠ MARGINAL (within 15%%)");
                } else {
                    telemetry.addData("Performance", "✗ POOR (>15%% error)");
                }
            }
            telemetry.addLine();

            // Velocity (raw encoder data)
            telemetry.addLine("=== VELOCITY (ticks/sec) ===");
            telemetry.addData("Target Velocity", "%.1f", targetVelocity);
            telemetry.addData("Actual Velocity", "%.1f", actualVelocity);
            telemetry.addLine();

            // Motor diagnostics
            telemetry.addLine("=== MOTOR DIAGNOSTICS ===");
            telemetry.addData("Motor Power", "%.3f", motorPower);
            telemetry.addData("Encoder Position", "%d", flywheelMotor.getCurrentPosition());

            // Battery voltage - LOW VOLTAGE SIGNIFICANTLY IMPACTS PERFORMANCE!
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            telemetry.addData("Battery Voltage", "%.2f V", voltage);
            if (voltage < 12.0) {
                telemetry.addLine("⚠ WARNING: Low battery affects max RPM!");
            }
            telemetry.addLine();

            // PIDF Info
            telemetry.addLine("=== PIDF COEFFICIENTS ===");
            telemetry.addData("P", FLYWHEEL_P);
            telemetry.addData("I", FLYWHEEL_I);
            telemetry.addData("D", FLYWHEEL_D);
            telemetry.addData("F", FLYWHEEL_F);
            telemetry.addLine();

            // Theoretical F calculation for current target
            double theoreticalF = 32767.0 / targetVelocity;
            telemetry.addData("Theoretical F for target", "%.2f", theoreticalF);
            telemetry.addData("Current F", "%.2f", FLYWHEEL_F);
            if (Math.abs(theoreticalF - FLYWHEEL_F) > 5) {
                telemetry.addLine("ℹ Consider adjusting F coefficient");
            }
            telemetry.addLine();

            // Controls reminder
            telemetry.addLine("=== CONTROLS ===");
            telemetry.addLine("A: Start | X: Stop");
            telemetry.addLine("DPAD ↑↓: Change RPM preset");

            telemetry.update();

            // Small delay to prevent overwhelming the system
            sleep(50);

        } // End of main loop

        /* ========================================
         * CLEANUP
         * ======================================== */
        flywheelMotor.setVelocity(0);
        hoodServo.setPosition(0.0);

    } // End of runOpMode

    /* ========================================
     * HELPER METHODS
     * ======================================== */

    /**
     * Convert RPM to velocity in ticks per second
     * @param rpm Target RPM
     * @return Velocity in ticks per second
     */
    private double rpmToVelocity(int rpm) {
        return (rpm / 60.0) * FLYWHEEL_CPR;
    }

    /**
     * Convert velocity (ticks per second) to RPM
     * @param velocity Velocity in ticks per second
     * @return RPM
     */
    private double velocityToRPM(double velocity) {
        return (velocity / FLYWHEEL_CPR) * 60.0;
    }

} // End of class
