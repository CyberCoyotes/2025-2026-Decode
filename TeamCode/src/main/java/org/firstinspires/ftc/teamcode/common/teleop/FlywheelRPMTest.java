package org.firstinspires.ftc.teamcode.common.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.subsystems.IndexSubsystem;

/**
 * Flywheel RPM Test OpMode
 *
 * Purpose: Diagnose why the GoBILDA 5203 Yellow Jacket motor (6000 RPM rated)
 * is not reaching higher RPM values (3000+, target 5000+).
 *
 * Tests three preset RPM values: 3000, 4000, and 5000 RPM
 * Hood is set to maximum position (0.6) for all tests
 *
 * INDEX MOTOR INTEGRATION:
 * - Index motors automatically run when flywheel is within 5% of target RPM
 * - Simulates actual shooting conditions with ball feed
 * - Helps test if mechanical load affects achievable RPM
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
 * - Index motor status (ready to feed/not ready)
 * - Battery voltage (affects performance)
 * - PIDF coefficients
 */
@Disabled
@TeleOp(name = "Flywheel RPM Test", group = "ALPHA")
public class FlywheelRPMTest extends LinearOpMode {

    // Hardware
    private DcMotorEx flywheelMotor;
    private Servo hoodServo;
    private IndexSubsystem indexSubsystem;

    // Hardware names - must match your robot configuration
    private static final String FLYWHEEL_MOTOR_NAME = "flywheelMotor";
    private static final String HOOD_SERVO_NAME = "hoodServo";

    // Motor specifications
    private static final double FLYWHEEL_CPR = 28.0; // Counts per revolution

    // RPM tolerance for index motor activation (5% matches ShooterSubsystem)
    private static final double RPM_TOLERANCE_PERCENT = 5.0;

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

        // Initialize hardware
        flywheelMotor = hardwareMap.get(DcMotorEx.class, FLYWHEEL_MOTOR_NAME);
        hoodServo = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);
        indexSubsystem = new IndexSubsystem(hardwareMap);

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
             * INDEX MOTOR CONTROL
             * ======================================== */
            // Calculate current performance
            int targetRPM = RPM_PRESETS[currentPresetIndex];
            double targetVelocity = rpmToVelocity(targetRPM);
            double actualVelocity = flywheelMotor.getVelocity();
            double actualRPM = velocityToRPM(actualVelocity);

            // Check if flywheel is at target RPM within tolerance
            boolean isAtTargetRPM = isAtTargetRPM(targetRPM, actualRPM);

            // Automatically run index motors when flywheel is ready
            if (isRunning && isAtTargetRPM) {
                indexSubsystem.runForward();
            } else {
                indexSubsystem.stop();
            }

            /* ========================================
             * TELEMETRY - DETAILED DIAGNOSTICS
             * ======================================== */
            telemetry.clearAll();

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

            // RPM Comparison with detailed analysis
            telemetry.addLine("=== RPM PERFORMANCE ===");
            telemetry.addData("Target RPM", "%d", targetRPM);
            telemetry.addData("Actual RPM", "%.1f", actualRPM);
            telemetry.addData("Error", "%.1f RPM (%.1f%%)", rpmError, errorPercent);

            // Calculate percentage of target achieved
            double percentAchieved = (actualRPM / targetRPM) * 100.0;
            telemetry.addData("% of Target Achieved", "%.1f%%", percentAchieved);

            // Visual indicator of how close we are
            if (isRunning) {
                if (Math.abs(errorPercent) < 5) {
                    telemetry.addData("Performance", "✓ GOOD (within 5%%)");
                } else if (Math.abs(errorPercent) < 15) {
                    telemetry.addData("Performance", "⚠ MARGINAL (within 15%%)");
                } else {
                    telemetry.addData("Performance", "✗ POOR (>15%% error)");
                }

                // Specific diagnosis for stuck power issue
                if (percentAchieved < 70.0) {
                    telemetry.addLine("⚠⚠ MAJOR ISSUE DETECTED:");
                    telemetry.addLine("  Motor not reaching target!");
                    telemetry.addLine("  Check:");
                    telemetry.addLine("  1. Battery voltage");
                    telemetry.addLine("  2. PIDF F coefficient");
                    telemetry.addLine("  3. Mechanical resistance");
                }
            }
            telemetry.addLine();

            // Index motor status
            telemetry.addLine("=== INDEX MOTORS ===");
            if (isAtTargetRPM && isRunning) {
                telemetry.addData("Status", "✓ FEEDING (RPM at target)");
            } else if (isRunning) {
                telemetry.addData("Status", "⏳ WAITING (Spinning up...)");
            } else {
                telemetry.addData("Status", "IDLE (Flywheel stopped)");
            }
            telemetry.addData("Bottom Motor", "%.2f", indexSubsystem.getBottomMotorPower());
            telemetry.addData("Top Motor", "%.2f", indexSubsystem.getTopMotorPower());
            telemetry.addLine();

            // Velocity (raw encoder data)
            telemetry.addLine("=== VELOCITY (ticks/sec) ===");
            telemetry.addData("Target Velocity", "%.1f", targetVelocity);
            telemetry.addData("Actual Velocity", "%.1f", actualVelocity);
            telemetry.addLine();

            // Motor diagnostics
            telemetry.addLine("=== MOTOR DIAGNOSTICS ===");
            telemetry.addData("Motor Power", "%.3f (%.1f%%)", motorPower, motorPower * 100);
            telemetry.addData("Encoder Position", "%d", flywheelMotor.getCurrentPosition());

            // CRITICAL: Diagnose why power isn't increasing with RPM
            if (isRunning) {
                double expectedMinPower = Math.min(1.0, targetRPM / 6000.0); // Rough estimate
                if (motorPower < expectedMinPower * 0.7) {
                    telemetry.addLine("⚠ POWER TOO LOW for target RPM!");
                    telemetry.addLine("  → PIDF tuning issue likely");
                }

                // Check if power is stuck/constant
                if (targetRPM >= 4000 && motorPower < 0.7) {
                    telemetry.addLine("⚠ Power should be >70% at 4000+ RPM");
                }
                if (targetRPM >= 5000 && motorPower < 0.85) {
                    telemetry.addLine("⚠ Power should be >85% at 5000 RPM");
                }
            }
            telemetry.addLine();

            // Battery voltage - LOW VOLTAGE SIGNIFICANTLY IMPACTS PERFORMANCE!
            double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
            telemetry.addData("Battery Voltage", "%.2f V", voltage);
            if (voltage < 12.0) {
                telemetry.addLine("⚠ WARNING: Low battery affects max RPM!");
            }
            if (voltage < 11.5) {
                telemetry.addLine("⚠⚠ CRITICAL: Battery too low!");
            }
            telemetry.addLine();

            // PIDF Info with actionable recommendations
            telemetry.addLine("=== PIDF ANALYSIS ===");
            telemetry.addData("Current P", "%.1f", FLYWHEEL_P);
            telemetry.addData("Current I", "%.1f", FLYWHEEL_I);
            telemetry.addData("Current D", "%.1f", FLYWHEEL_D);
            telemetry.addData("Current F", "%.1f", FLYWHEEL_F);
            telemetry.addLine();

            // Theoretical F calculation for current target
            double theoreticalF = 32767.0 / targetVelocity;
            telemetry.addData("Theoretical F (optimal)", "%.2f", theoreticalF);
            telemetry.addData("F Difference", "%.2f", FLYWHEEL_F - theoreticalF);

            if (Math.abs(theoreticalF - FLYWHEEL_F) > 5) {
                if (FLYWHEEL_F > theoreticalF + 5) {
                    telemetry.addLine("⚠ F is TOO HIGH for this RPM");
                    telemetry.addData("  Recommended F", "%.1f", theoreticalF);
                } else {
                    telemetry.addLine("⚠ F is TOO LOW for this RPM");
                    telemetry.addData("  Recommended F", "%.1f", theoreticalF);
                }
            } else {
                telemetry.addLine("✓ F is appropriate for target");
            }

            // Show recommended PIDF for all presets
            telemetry.addLine();
            telemetry.addLine("=== RECOMMENDED PIDF VALUES ===");
            for (int i = 0; i < RPM_PRESETS.length; i++) {
                int rpm = RPM_PRESETS[i];
                double vel = rpmToVelocity(rpm);
                double recF = 32767.0 / vel;
                String marker = (i == currentPresetIndex) ? "→ " : "  ";
                telemetry.addData(marker + rpm + " RPM", "F=%.1f", recF);
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
        indexSubsystem.stop();

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

    /**
     * Check if actual RPM is within acceptable tolerance of target RPM
     * @param targetRPM Target RPM
     * @param actualRPM Actual RPM measured from encoder
     * @return true if within tolerance
     */
    private boolean isAtTargetRPM(int targetRPM, double actualRPM) {
        if (targetRPM == 0) {
            return Math.abs(actualRPM) < 100.0; // Consider stopped if < 100 RPM
        }
        double tolerance = targetRPM * (RPM_TOLERANCE_PERCENT / 100.0);
        return Math.abs(actualRPM - targetRPM) <= tolerance;
    }

} // End of class
