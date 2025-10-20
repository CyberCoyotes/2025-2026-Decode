# Limelight 3A Integration Architecture

## System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Control Hub                               │
│                                                              │
│  ┌────────────────────────────────────────────────────┐    │
│  │           RobotContainer (TeleOp)                  │    │
│  │                                                     │    │
│  │  Initialization:                                   │    │
│  │    limelightSub = new LimelightSubsystem(...)      │    │
│  │                                                     │    │
│  │  Main Loop:                                        │    │
│  │    limelightSub.update()                           │    │
│  │    telemetry.addData("AprilTags", ...)             │    │
│  │                                                     │    │
│  │  Cleanup:                                          │    │
│  │    limelightSub.stop()                             │    │
│  └─────────────────┬──────────────────────────────────┘    │
│                    │                                        │
│                    │ uses                                   │
│                    ▼                                        │
│  ┌────────────────────────────────────────────────────┐    │
│  │        LimelightSubsystem                          │    │
│  │                                                     │    │
│  │  Methods:                                          │    │
│  │    • update()                - Refresh data        │    │
│  │    • getDetectedAprilTags()  - Get all tags       │    │
│  │    • getAprilTagCount()      - Count tags         │    │
│  │    • isTagDetected(id)       - Check for tag      │    │
│  │    • getTagById(id)          - Get specific tag   │    │
│  │    • getTargetX/Y()          - Get angles         │    │
│  │    • getBotPose()            - Get position       │    │
│  │    • getStatusTelemetry()    - Status string      │    │
│  │    • getAprilTagTelemetry()  - Tag info string    │    │
│  └─────────────────┬──────────────────────────────────┘    │
│                    │                                        │
│                    │ controls                               │
│                    ▼                                        │
│  ┌────────────────────────────────────────────────────┐    │
│  │          Limelight3A Hardware                      │    │
│  │                                                     │    │
│  │  SDK Methods:                                      │    │
│  │    • start()                                       │    │
│  │    • stop()                                        │    │
│  │    • getLatestResult()                             │    │
│  │    • getStatus()                                   │    │
│  │    • pipelineSwitch(index)                         │    │
│  └─────────────────┬──────────────────────────────────┘    │
│                    │                                        │
└────────────────────┼────────────────────────────────────────┘
                     │ USB Connection
                     ▼
           ┌──────────────────────┐
           │   Limelight 3A       │
           │   Vision Sensor      │
           │                      │
           │  IP: 192.168.1.11    │
           │  Pipeline 0: 36h11   │
           └──────────────────────┘
                     │
                     │ Observes
                     ▼
           ┌──────────────────────┐
           │   AprilTags          │
           │   (36h11 family)     │
           │                      │
           │   ID: 1, 2, 3...     │
           └──────────────────────┘
```

## Data Flow

### 1. Initialization (Once at Start)
```
OpMode Start
    ├─> Create LimelightSubsystem
    │   ├─> Get Limelight3A from hardwareMap
    │   ├─> Switch to pipeline 0 (36h11)
    │   └─> Start polling
    └─> Ready to detect
```

### 2. Main Loop (Continuous)
```
Loop Iteration
    ├─> limelightSub.update()
    │   ├─> Get latest result from hardware
    │   └─> Get status from hardware
    │
    ├─> Check hasValidData()
    │   └─> If valid:
    │       ├─> Get detected tags list
    │       ├─> Extract IDs, positions
    │       └─> Calculate angles
    │
    └─> Display on telemetry
        ├─> Status (temp, CPU, FPS)
        ├─> AprilTag count and IDs
        ├─> Target angles
        └─> Latency
```

### 3. Cleanup (On Stop)
```
OpMode Stop
    └─> limelightSub.stop()
        └─> Stop Limelight polling
```

## File Responsibilities

### LimelightSubsystem.java
**Purpose**: Abstraction layer for Limelight hardware  
**Responsibilities**:
- Initialize and configure Limelight
- Wrap SDK calls with friendly API
- Process and filter tag data
- Format telemetry strings
- Handle null/invalid data safely

### RobotContainer.java
**Purpose**: Main TeleOp OpMode  
**Responsibilities**:
- Create subsystem instances
- Call update methods in loop
- Display telemetry to drivers
- Clean up on exit

### TestLimelight.java
**Purpose**: Standalone test/diagnostic  
**Responsibilities**:
- Independent verification
- Detailed status display
- Troubleshooting aid
- No dependencies on other subsystems

### LIMELIGHT_SETUP.md
**Purpose**: Complete setup documentation  
**Contents**:
- Hardware setup steps
- Software configuration
- Pipeline configuration
- Testing procedures
- Troubleshooting guide

### LIMELIGHT_QUICK_START.md
**Purpose**: Quick reference guide  
**Contents**:
- 5-minute setup
- Code examples
- Common issues
- Next steps

## Tag Detection Process

```
Limelight Camera
    │
    ├─> Capture Frame (90 FPS typical)
    │
    ├─> Run AprilTag Detector
    │   ├─> Use 36h11 dictionary
    │   ├─> Detect tag corners
    │   ├─> Extract ID from pattern
    │   └─> Calculate pose (position/rotation)
    │
    ├─> Bundle Results (LLResult)
    │   ├─> List of detected tags
    │   ├─> Tag IDs and families
    │   ├─> 3D positions (x, y, z)
    │   ├─> Angles (tx, ty)
    │   └─> Robot pose (if available)
    │
    └─> Send to Control Hub
        │
        └─> Available via getLatestResult()
```

## Configuration Requirements

### Hardware Map
```xml
<!-- In robot configuration -->
<Limelight3A name="limelight" serialNumber="192.168.1.xxx" />
```

### Limelight Pipeline 0
- **Type**: AprilTag (Fiducial)
- **Family**: 36h11
- **Decimation**: 2.0
- **Threads**: 3
- **Refine Edges**: Enabled

## Integration Points

### Telemetry Display
```
--- LIMELIGHT ---
Status: Temp: XX.X°C, CPU: XX.X%, FPS: XX, Pipeline: 0
AprilTags: Detected N tag(s): ID X (XX.X°, XX.X°), ...
Target Angle: X: XX.X°, Y: XX.X°
Latency: XX.X ms
```

### Autonomous Usage (Future)
```java
// Example: Align to tag 5
if (limelightSub.isTagDetected(5)) {
    double tx = limelightSub.getTargetX();
    // Use tx for alignment control
    drive.turn(tx * alignmentGain);
}
```

### Navigation (Future)
```java
// Example: Use botpose for localization
Pose3D pose = limelightSub.getBotPose();
if (pose != null) {
    // Update odometry with vision data
    drive.updatePoseFromVision(pose);
}
```

## Error Handling Strategy

### Subsystem Level
- Always check `hasValidData()` before accessing results
- Return empty lists instead of null for safety
- Return 0.0 for numeric values when invalid
- Provide status methods for diagnostics

### OpMode Level
- Display "No data available" when invalid
- Show diagnostic info when no tags detected
- Continue operation even without Limelight data
- Gracefully handle initialization failures

## Performance Considerations

### Loop Timing
- `update()` is fast (< 1ms typically)
- Result processing is minimal
- Telemetry formatting is string-based
- No blocking operations

### Resource Usage
- Limelight runs on separate processor
- Control Hub CPU: ~25-35% typical
- USB bandwidth: < 10 Mbps
- Power draw: < 5W from USB

## Next Steps for Teams

1. **Immediate**: Test basic detection
2. **Short-term**: Add alignment behaviors
3. **Medium-term**: Integrate with autonomous
4. **Long-term**: Use for localization

## Support Resources

- **Code Samples**: FtcRobotController/samples/SensorLimelight3A.java
- **Limelight Docs**: https://limelightvision.io/
- **FTC Docs**: https://ftc-docs.firstinspires.org/
- **AprilTag Info**: https://github.com/AprilRobotics/apriltag
