package org.firstinspires.ftc.teamcode.common.helpers;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Collections;
import java.util.List;

/**
 * Plain helper wrapping the Limelight 3A vision camera for Teams 11940 &amp; 22091.
 *
 * <p>This is a <strong>helper class</strong>, not a subsystem. The Limelight is pure-read sensor
 * hardware — nothing ever commands it, so it does not extend {@code SubsystemBase} and is not
 * registered with the FTCLib scheduler. Instead, the OpMode calls {@link #update()} once per loop
 * tick (typically right after {@code pinpoint.update()}) to pull the latest result and status
 * snapshots from the SDK.</p>
 *
 * <p><strong>Typical wiring in TeleOp:</strong>
 * <pre>
 *   Limelight limelight = new Limelight(hardwareMap);
 *   // ... in loop:
 *   limelight.update();
 *   if (limelight.hasTarget()) { ... limelight.getTx() ... }
 * </pre>
 * </p>
 *
 * <p><strong>Vision-pipeline state</strong> is exposed via {@link #getStatus()}, which returns the
 * {@link Status} enum ({@code OFFLINE}, {@code SEARCHING}, {@code LOCKED}). Use
 * {@link #hasTarget()} / {@link #isLocked()} as convenience predicates for the common case.</p>
 *
 * <p><strong>Hardware:</strong>
 * <ul>
 *   <li>Hardware config name: {@value #LIMELIGHT_NAME} — do not change without driver-station
 *       reconfiguration (ARCHITECTURE.md §5)</li>
 *   <li>Pipeline 0 = AprilTag 36h11 (configured in the Limelight web interface, not in code)</li>
 *   <li>IP: 192.168.1.11</li>
 * </ul>
 * </p>
 *
 * <p><strong>Pose fusion:</strong> {@link #getBotPose()} and {@link #getTotalLatencyMs()} are the
 * two values consumed by the latency-compensated Pinpoint + Limelight fusion described in
 * ARCHITECTURE.md §7.</p>
 */
public class Limelight {

    // Hardware config name (ARCHITECTURE.md §5)
    private static final String LIMELIGHT_NAME = "limelight";

    // Pipeline 0 = AprilTag 36h11 (configured in the Limelight web interface, not in code)
    private static final int DEFAULT_PIPELINE = 0;

    private static final int POLL_RATE_HZ = 100;

    public enum Status {
        OFFLINE,
        SEARCHING,
        LOCKED
    }

    private final Limelight3A limelight;
    private LLResult           latestResult;
    private LLStatus           latestStatus;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(DEFAULT_PIPELINE);
        limelight.setPollRateHz(POLL_RATE_HZ);
        limelight.start();
    }

    /**
     * Pull the latest result and status snapshots from the Limelight SDK.
     *
     * <p>Call once per OpMode loop tick, typically right after {@code pinpoint.update()}.
     * All getters reflect the values captured during the most recent {@code update()} call.</p>
     */
    public void update() {
        latestResult = limelight.getLatestResult();
        latestStatus = limelight.getStatus();
    }

    // ---- Status -------------------------------------------------------

    /**
     * Derives the current operating state from the most recent SDK snapshots.
     *
     * <p>OFFLINE is signaled by a null {@code LLStatus} (pre-first-poll case) OR by
     * {@code getTemp() <= 0.0} (unreachable-device case — a running camera always reports
     * positive thermal data). This temperature check is based on observed SDK behavior.
     *
     * <p>TODO: verify temperature-based OFFLINE detection against physical hardware.
     * If OFFLINE triggers spuriously during normal operation, switch to another signal
     * (e.g. stale FPS reading, status timestamp) as needed.
     */
    public Status getStatus() {
        if (latestStatus == null || latestStatus.getTemp() <= 0.0) return Status.OFFLINE;
        if (latestResult == null || !latestResult.isValid())        return Status.SEARCHING;
        return Status.LOCKED;
    }

    public boolean isOffline()   { return getStatus() == Status.OFFLINE; }
    public boolean isSearching() { return getStatus() == Status.SEARCHING; }
    public boolean isLocked()    { return getStatus() == Status.LOCKED; }

    /** Alias for {@link #isLocked()}; reads more naturally at call sites. */
    public boolean hasTarget()   { return isLocked(); }

    // ---- Getters ------------------------------------------------------

    /**
     * Horizontal angle to the primary target, in degrees.
     *
     * <p><strong>Caller must verify {@link #isLocked()} (or {@link #hasTarget()}) before
     * reading this value.</strong> When not locked, the return value is undefined — in
     * practice 0.0 or whatever the Limelight last reported. This is the standard FRC/FTC
     * convention for tx/ty; the contract is the guard, not a sentinel return value.
     */
    public double getTx() {
        return latestResult != null ? latestResult.getTx() : 0.0;
    }

    /**
     * Vertical angle to the primary target, in degrees.
     *
     * <p>Same contract as {@link #getTx()} — caller must verify {@link #isLocked()} first.
     */
    public double getTy() {
        return latestResult != null ? latestResult.getTy() : 0.0;
    }

    /**
     * Robot's estimated field pose derived from visible AprilTags.
     *
     * @return Pose3D from the most recent valid result, or {@code null} if not locked.
     */
    public Pose3D getBotPose() {
        if (latestResult == null || !latestResult.isValid()) return null;
        return latestResult.getBotpose();
    }

    /**
     * Total vision-pipeline latency (capture + targeting), in milliseconds.
     *
     * <p>Returns 0.0 when no valid result is available. When implementing pose fusion
     * (ARCHITECTURE.md §7), use this value: the pose returned by {@link #getBotPose()} is
     * this many milliseconds old and should be combined with drive state at the capture
     * timestamp, not the current timestamp.
     *
     * @return latency in ms, or 0.0 if not locked.
     */
    public double getTotalLatencyMs() {
        if (latestResult == null || !latestResult.isValid()) return 0.0;
        return latestResult.getCaptureLatency() + latestResult.getTargetingLatency();
    }

    /**
     * All AprilTags visible in the current frame.
     *
     * @return list of fiducial results; never {@code null}; empty when not locked.
     */
    public List<LLResultTypes.FiducialResult> getDetectedAprilTags() {
        if (latestResult == null || !latestResult.isValid()) return Collections.emptyList();
        return latestResult.getFiducialResults();
    }

    /**
     * Finds a specific AprilTag by its fiducial ID.
     *
     * @param tagId fiducial ID to search for.
     * @return the matching {@link LLResultTypes.FiducialResult}, or {@code null} if not detected.
     */
    public LLResultTypes.FiducialResult getTagById(int tagId) {
        for (LLResultTypes.FiducialResult tag : getDetectedAprilTags()) {
            if (tag.getFiducialId() == tagId) return tag;
        }
        return null;
    }

    /**
     * Currently active pipeline index as reported by the Limelight.
     *
     * @return pipeline index, or -1 if the status snapshot is not yet available.
     */
    public int getPipelineIndex() {
        return latestStatus != null ? latestStatus.getPipelineIndex() : -1;
    }

    /**
     * Raw {@link LLStatus} snapshot, for telemetry and diagnostics only.
     *
     * <p>Exposes internal SDK state. Use only for debugging or telemetry display —
     * do not branch on {@code LLStatus} fields in production logic.
     *
     * @return the most recent status snapshot, or {@code null} before the first {@link #update()} call.
     */
    public LLStatus getLLStatus() {
        return latestStatus;
    }

    // ---- Verb methods -------------------------------------------------

    /**
     * Switches the active vision pipeline.
     *
     * <p>Fire-and-forget: returns immediately; the Limelight changes pipelines
     * asynchronously within milliseconds.
     *
     * @param pipelineIndex pipeline to activate (0–9).
     */
    public void switchPipeline(int pipelineIndex) {
        limelight.pipelineSwitch(pipelineIndex);
    }
}
