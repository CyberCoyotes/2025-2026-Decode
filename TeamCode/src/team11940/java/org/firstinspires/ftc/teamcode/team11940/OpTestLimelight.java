package org.firstinspires.ftc.teamcode.team11940;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// Robot Specific Imports
import org.firstinspires.ftc.teamcode.common.subsystems.LimelightSubsystem;

import java.util.List;

/**
 * Test OpMode for the Limelight 3A AprilTag detection
 * This is a standalone test to verify Limelight functionality
 * Configure the Limelight in the robot configuration as "limelight"
 * Pipeline 0 should be configured for AprilTag detection with 36h11 family
 */
@TeleOp(name = "Limelight AprilTag", group = "ALPHA")
@Disabled

public class OpTestLimelight extends LinearOpMode {

    private LimelightSubsystem limelightSub;

    @Override
    public void runOpMode() throws InterruptedException {
        
        telemetry.addData("Status", "Initializing Limelight...");
        telemetry.update();

        // Initialize the Limelight subsystem
        limelightSub = new LimelightSubsystem(hardwareMap);

        telemetry.addData("Status", "Limelight initialized. Ready to start.");
        telemetry.addLine();
        telemetry.addLine("This OpMode will display:");
        telemetry.addLine("- Limelight status (temp, CPU, FPS)");
        telemetry.addLine("- Detected AprilTag IDs");
        telemetry.addLine("- Tag positions and angles");
        telemetry.addLine();
        telemetry.addLine("Make sure pipeline 0 is configured");
        telemetry.addLine("for 36h11 AprilTag family.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update Limelight data
            limelightSub.update();

            telemetry.clearAll();
            telemetry.addLine("=== LIMELIGHT 3A TEST ===");
            telemetry.addLine();

            // Display Limelight status
            telemetry.addLine("--- STATUS ---");
            telemetry.addData("Device Status", limelightSub.getStatusTelemetry());
            telemetry.addLine();

            // Display AprilTag detection results
            telemetry.addLine("--- APRILTAG DETECTION ---");
            telemetry.addData("Valid Data", limelightSub.hasValidData() ? "YES" : "NO");
            
            if (limelightSub.hasValidData()) {
                int tagCount = limelightSub.getAprilTagCount();
                telemetry.addData("Tags Detected", tagCount);
                
                if (tagCount > 0) {
                    telemetry.addLine();
                    telemetry.addLine("--- DETECTED TAGS ---");
                    
                    List<LLResultTypes.FiducialResult> tags = limelightSub.getDetectedAprilTags();
                    for (int i = 0; i < tags.size(); i++) {
                        LLResultTypes.FiducialResult tag = tags.get(i);
                        telemetry.addLine(String.format("Tag #%d:", i + 1));
                        telemetry.addData("  ID", tag.getFiducialId());
                        telemetry.addData("  Family", tag.getFamily());
                        telemetry.addData("  X Angle", String.format("%.2f°", tag.getTargetXDegrees()));
                        telemetry.addData("  Y Angle", String.format("%.2f°", tag.getTargetYDegrees()));
                        telemetry.addData("  Area", String.format("%.2f%%", tag.getTargetArea()));
                    }
                    
                    telemetry.addLine();
                    telemetry.addLine("--- TARGETING INFO ---");
                    telemetry.addData("Primary Target X", String.format("%.2f°", limelightSub.getTargetX()));
                    telemetry.addData("Primary Target Y", String.format("%.2f°", limelightSub.getTargetY()));
                    telemetry.addData("Total Latency", String.format("%.1f ms", limelightSub.getTotalLatency()));
                } else {
                    telemetry.addLine();
                    telemetry.addData("Info", "No AprilTags detected");
                    telemetry.addLine("Point camera at 36h11 tags");
                }
            } else {
                telemetry.addData("Warning", "No valid data from Limelight");
                telemetry.addLine("Check:");
                telemetry.addLine("- Limelight is powered on");
                telemetry.addLine("- USB connection is secure");
                telemetry.addLine("- Robot configuration is correct");
            }

            telemetry.update();

            // Small delay to reduce CPU usage
            sleep(50);
        }

        // Clean up
//        limelightSub.stop();
        
        telemetry.addData("Status", "OpMode stopped");
        telemetry.update();
    }
}
