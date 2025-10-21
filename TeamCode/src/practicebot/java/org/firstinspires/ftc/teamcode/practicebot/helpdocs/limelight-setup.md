# Limelight 3A Setup Instructions for FTC

This document provides step-by-step instructions for setting up the Limelight 3A vision sensor for FTC use with the practice bot.

## Hardware Setup

### 1. Initial Connection
1. Connect the Limelight 3A to a USB port on the Control Hub using the provided USB cable
2. The Limelight will appear as an ethernet interface when plugged in
3. A DHCP server running on the Limelight will automatically assign an IP address to the Control Hub

### 2. Power Considerations
- The Limelight 3A draws power from the USB port
- Ensure your Control Hub has adequate power supply
- Monitor battery levels during testing

## Software Configuration

### 3. Configure in Robot Controller App

1. Open the Robot Controller app on the Control Hub
2. Navigate to the Configuration menu (gear icon)
3. The Limelight will be listed as a USB device on the top level configuration screen
4. You'll see the Control Hub's assigned IP address displayed below the device name (this acts as a "serial number")
5. Tap on the Limelight device to open its configuration screen
6. Rename the device to "limelight" (this name must match the hardware map name in your code)
7. Enter the Limelight's IP address:
   - Default Limelight IP address: `192.168.1.11`
   - Do NOT confuse this with the IP address the Limelight assigned to the Control Hub
8. Save the configuration

### 4. Configure AprilTag Pipeline via Web Interface

1. Connect a laptop to the same network as the Control Hub
2. Open a web browser and navigate to `http://192.168.1.11:5801` (or your Limelight's IP)
3. This opens the Limelight configuration interface
4. Navigate to the Pipeline settings
5. Create a new pipeline or modify an existing one for AprilTag detection:
   - **Pipeline Type**: AprilTag (Fiducial)
   - **Tag Family**: 36h11 (for 2025-2026 FTC DECODE season)
   - **Decimation**: 2.0 (good balance between speed and accuracy)
   - **Blur**: 0.0 (no blur needed typically)
   - **Threads**: 3 (use multiple threads for faster detection)
   - **Refine Edges**: Enabled (improves detection accuracy)
   - **Tag Size**: Set according to FTC specifications (varies by game piece)
6. Save the pipeline settings
7. Note the pipeline index (typically 0 if it's your first pipeline)

### 5. Verify Basic Operation

Before running code:
1. Look at the Limelight web interface video feed
2. Point the camera at an AprilTag (36h11 family)
3. You should see the tag outlined in the video feed
4. Verify the tag ID is correctly identified

## Code Integration

### 6. Hardware Map Configuration

The Limelight must be configured in your robot configuration with the name `"limelight"` to match the code:

```java
limelight = hardwareMap.get(Limelight3A.class, "limelight");
```

### 7. Pipeline Selection

The code defaults to pipeline 0. If you created a different pipeline, update this line:

```java
limelight.pipelineSwitch(0);  // Change 0 to your pipeline index
```

### 8. Start the Limelight

The Limelight must be started to begin polling for data:

```java
limelight.start();
```

### 9. Read AprilTag Data

Access detected AprilTags (fiducials):

```java
LLResult result = limelight.getLatestResult();
if (result.isValid()) {
    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
    for (LLResultTypes.FiducialResult fr : fiducialResults) {
        int tagId = fr.getFiducialId();
        String family = fr.getFamily();
        // Use the tag data...
    }
}
```

## Testing

### 10. Initial Test

1. Deploy the practice bot code with Limelight support
2. Initialize the OpMode
3. Check telemetry on the Driver Station for:
   - Limelight status (temperature, CPU, FPS)
   - Pipeline information
   - Detected AprilTag IDs
   - Tag positions (tx, ty angles)

### 11. Verification Checklist

- [ ] Limelight appears in robot configuration
- [ ] Limelight web interface accessible
- [ ] Pipeline configured for 36h11 tags
- [ ] Tags detected in web interface video feed
- [ ] OpMode initializes without errors
- [ ] Telemetry shows Limelight status
- [ ] AprilTag IDs appear in telemetry when tags are visible
- [ ] Tag detection is reliable and consistent

## Troubleshooting

### Common Issues

**Limelight not appearing in configuration:**
- Check USB connection
- Try a different USB port
- Restart the Control Hub

**No data in telemetry:**
- Verify `limelight.start()` is called
- Check pipeline is set correctly
- Ensure the Limelight has powered up (takes ~10-15 seconds)
- Check that `isValid()` returns true before accessing data

**AprilTags not detected:**
- Verify tag family is 36h11
- Check lighting conditions (avoid harsh shadows or glare)
- Ensure tags are properly printed and flat
- Check tag size configuration in pipeline
- Verify camera is in focus

**Performance issues:**
- Reduce decimation (increases processing time but improves detection)
- Use fewer threads if Control Hub is overloaded
- Disable features not needed in the pipeline

## Resources

- [Limelight Documentation](https://limelightvision.io/)
- [FTC Documentation](https://ftc-docs.firstinspires.org/)
- [AprilTag Library](https://github.com/AprilRobotics/apriltag)
- Sample Code: `SensorLimelight3A.java` in FtcRobotController samples

## Notes

- The Limelight 3A supports multiple pipeline types (AprilTag, color, neural network, etc.)
- For DECODE season (2025-2026), we use 36h11 tag family exclusively
- Limelight provides botpose (robot position) data that can be used for localization
- Consider adding a visual indicator LED or sound when tags are detected
- The Limelight's web interface can be used to tune detection parameters in real-time
