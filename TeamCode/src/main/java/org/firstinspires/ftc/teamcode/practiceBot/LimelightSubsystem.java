package org.firstinspires.ftc.teamcode.practiceBot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Subsystem for managing the Limelight 3A vision sensor
 * Configured for AprilTag detection using 36h11 tag family (2025-2026 FTC DECODE season)
 */
public class LimelightSubsystem {

    private Limelight3A limelight;
    private LLResult latestResult;
    private LLStatus status;

    /**
     * Initialize the Limelight subsystem
     * @param hardwareMap The hardware map from the OpMode
     */
    public LimelightSubsystem(HardwareMap hardwareMap) {
        // Get the Limelight from hardware map
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        
        // Switch to pipeline 0 (configure this in Limelight web interface for 36h11 AprilTags)
        limelight.pipelineSwitch(0);
        
        // Start polling for data
        limelight.start();
    }

    /**
     * Update the latest result and status from the Limelight
     * Call this periodically in your main loop
     */
    public void update() {
        latestResult = limelight.getLatestResult();
        status = limelight.getStatus();
    }

    /**
     * Get the latest result from the Limelight
     * @return LLResult object containing detection data
     */
    public LLResult getLatestResult() {
        return latestResult;
    }

    /**
     * Get the status of the Limelight (temperature, CPU, FPS, etc.)
     * @return LLStatus object containing status information
     */
    public LLStatus getStatus() {
        return status;
    }

    /**
     * Check if the Limelight has valid data
     * @return true if data is valid, false otherwise
     */
    public boolean hasValidData() {
        return latestResult != null && latestResult.isValid();
    }

    /**
     * Get all detected AprilTags (fiducials)
     * @return List of FiducialResult objects, or empty list if no tags detected
     */
    public List<LLResultTypes.FiducialResult> getDetectedAprilTags() {
        if (hasValidData()) {
            return latestResult.getFiducialResults();
        }
        return java.util.Collections.emptyList();
    }

    /**
     * Get the number of AprilTags currently detected
     * @return Number of detected tags
     */
    public int getAprilTagCount() {
        return getDetectedAprilTags().size();
    }

    /**
     * Check if a specific AprilTag ID is currently detected
     * @param targetId The AprilTag ID to look for
     * @return true if the tag is detected, false otherwise
     */
    public boolean isTagDetected(int targetId) {
        for (LLResultTypes.FiducialResult tag : getDetectedAprilTags()) {
            if (tag.getFiducialId() == targetId) {
                return true;
            }
        }
        return false;
    }

    /**
     * Get a specific AprilTag by ID
     * @param targetId The AprilTag ID to find
     * @return FiducialResult for the tag, or null if not found
     */
    public LLResultTypes.FiducialResult getTagById(int targetId) {
        for (LLResultTypes.FiducialResult tag : getDetectedAprilTags()) {
            if (tag.getFiducialId() == targetId) {
                return tag;
            }
        }
        return null;
    }

    /**
     * Get the robot's estimated position (botpose) from AprilTag localization
     * @return Pose3D object with robot position, or null if not available
     */
    public Pose3D getBotPose() {
        if (hasValidData()) {
            return latestResult.getBotpose();
        }
        return null;
    }

    /**
     * Get the horizontal angle to the primary target
     * @return Horizontal angle in degrees (tx)
     */
    public double getTargetX() {
        if (hasValidData()) {
            return latestResult.getTx();
        }
        return 0.0;
    }

    /**
     * Get the vertical angle to the primary target
     * @return Vertical angle in degrees (ty)
     */
    public double getTargetY() {
        if (hasValidData()) {
            return latestResult.getTy();
        }
        return 0.0;
    }

    /**
     * Get the total latency (capture + targeting)
     * @return Latency in milliseconds
     */
    public double getTotalLatency() {
        if (hasValidData()) {
            return latestResult.getCaptureLatency() + latestResult.getTargetingLatency();
        }
        return 0.0;
    }

    /**
     * Switch to a different pipeline
     * @param pipelineIndex The index of the pipeline to switch to (0-9)
     */
    public void switchPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }

    /**
     * Stop the Limelight
     * Call this when the OpMode is stopping
     */
    public void stop() {
        limelight.stop();
    }

    /**
     * Get a formatted string with AprilTag information for telemetry
     * @return String with AprilTag data
     */
    public String getAprilTagTelemetry() {
        StringBuilder sb = new StringBuilder();
        List<LLResultTypes.FiducialResult> tags = getDetectedAprilTags();
        
        if (tags.isEmpty()) {
            sb.append("No AprilTags detected");
        } else {
            sb.append(String.format("Detected %d tag(s): ", tags.size()));
            for (int i = 0; i < tags.size(); i++) {
                LLResultTypes.FiducialResult tag = tags.get(i);
                if (i > 0) sb.append(", ");
                sb.append(String.format("ID %d (%.1f°, %.1f°)", 
                    tag.getFiducialId(), 
                    tag.getTargetXDegrees(), 
                    tag.getTargetYDegrees()));
            }
        }
        
        return sb.toString();
    }

    /**
     * Get Limelight status information for telemetry
     * @return String with status data
     */
    public String getStatusTelemetry() {
        if (status == null) {
            return "Status: Unknown";
        }
        return String.format("Temp: %.1f°C, CPU: %.1f%%, FPS: %d, Pipeline: %d", 
            status.getTemp(), 
            status.getCpu(), 
            (int)status.getFps(),
            status.getPipelineIndex());
    }
}
