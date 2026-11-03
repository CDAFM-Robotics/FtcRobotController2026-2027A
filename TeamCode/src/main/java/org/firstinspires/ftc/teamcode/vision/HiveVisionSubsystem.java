package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Detects the state of the Hive field element using AprilTag vision.
 *
 * HOW IT WORKS:
 *   The Hive is a bi-stable pivot element — it rests with either its
 *   non-audience face up or its audience face up. Each face has a cluster
 *   of 4 AprilTags registered as a named group in the FTC game set.
 *
 *   This subsystem samples which tag cluster is visible every 0.5 seconds
 *   and keeps a rolling history of the last 4 samples to determine whether
 *   the Hive is stable (up on one side) or mid-tip.
 *
 * TO USE IN YOUR OPMODE:
 *   1. Construct:  hive = new HiveVisionSubsystem(hardwareMap, Alliance.RED, RobotSide.NON_AUDIENCE);
 *   2. Each loop:  hive.update();
 *   3. Read state: hive.getState()  → returns a HiveState enum value
 *   4. Debug:      hive.addTelemetry(telemetry);
 *   5. At end:     hive.close();
 */
public class HiveVisionSubsystem {

    /** Which alliance we're on — determines which AprilTag IDs to look for. */
    public enum Alliance { RED, BLUE }

    /**
     * Which side of the field the robot starts on.
     * This is used as a fallback assumption when no tags are visible:
     * if you're on the NON_AUDIENCE side, assume non-audience face is up (you'd see it).
     */
    public enum RobotSide { AUDIENCE, NON_AUDIENCE }

    /** One data point: which face appears higher (or no tags seen at all). */
    private enum Sample { NON_AUDIENCE_HIGHER, AUDIENCE_HIGHER, NO_DETECTION }

    // -----------------------------------------------------------------------
    // TUNING CONSTANTS — adjust these if detection feels too slow or jittery
    // -----------------------------------------------------------------------

    /** How many samples to keep in history. More = smoother but slower to react. */
    private static final int HISTORY_SIZE = 4;

    /** How often (in seconds) to take a new sample. Lower = more responsive. */
    private static final double SAMPLE_INTERVAL_SEC = 0.5;

    private final Alliance alliance;
    private final RobotSide robotSide;

    /** AprilTag IDs for the non-audience face cluster. Set in the constructor. */
    private final int[] nonAudienceIds;

    /** AprilTag IDs for the audience face cluster. Set in the constructor. */
    private final int[] audienceIds;

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;
    private final ElapsedTime sampleTimer = new ElapsedTime();

    private final Sample[] history = new Sample[HISTORY_SIZE];
    private int historyCount = 0;

    // Telemetry snapshots from last sample
    private List<Integer> lastNonAudienceIds = new ArrayList<>();
    private List<Integer> lastAudienceIds    = new ArrayList<>();
    private List<String>  lastClusterNames   = new ArrayList<>();
    private double lastNonAudienceZ = Double.NaN;
    private double lastAudienceZ    = Double.NaN;
    private Sample lastSample = Sample.NO_DETECTION;

    /**
     * @param hardwareMap  from your OpMode — used to find the webcam
     * @param alliance     RED or BLUE — picks the correct set of tag IDs
     * @param robotSide    where the robot starts — used as a fallback when no tags are visible
     *
     * TAG ID REFERENCE (BIOBUZZ 2026-2027):
     *   RED  non-audience face (cluster "RED_SCORING"):  30, 31, 32, 33
     *   RED  audience face    (cluster "RED_AUDIENCE"):  34, 35, 36, 37
     *   BLUE audience face    (cluster "BLUE_AUDIENCE"): 38, 39, 40, 41
     *   BLUE non-audience face (cluster "BLUE_SCORING"): 42, 43, 44, 45
     */
    public HiveVisionSubsystem(HardwareMap hardwareMap, Alliance alliance, RobotSide robotSide) {
        this.alliance = alliance;
        this.robotSide = robotSide;

        // Assign tag ID arrays based on alliance
        if (alliance == Alliance.RED) {
            nonAudienceIds = new int[]{30, 31, 32, 33};
            audienceIds    = new int[]{34, 35, 36, 37};
        } else {
            nonAudienceIds = new int[]{42, 43, 44, 45};
            audienceIds    = new int[]{38, 39, 40, 41};
        }

        // Pre-fill history so getState() doesn't return UNKNOWN immediately
        Arrays.fill(history, Sample.NO_DETECTION);

        // easyCreateWithDefaults() uses the 36h11 tag family, which matches the FTC game set
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        sampleTimer.reset();
    }

    /**
     * Call this every loop iteration. It checks the camera state and takes
     * a new sample at each SAMPLE_INTERVAL_SEC.
     *
     * Nothing happens until the camera finishes initializing (state = STREAMING),
     * so it's safe to call update() right away after construction.
     */
    public void update() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return;
        if (sampleTimer.seconds() >= SAMPLE_INTERVAL_SEC) {
            sampleTimer.reset();
            takeSample();
        }
    }

    /**
     * Call this once at the START of autonomous (after the camera is streaming)
     * to pre-fill the history with the current observation. Without this,
     * getState() returns UNKNOWN for the first 2 seconds while history fills up.
     */
    public void initSample() {
        takeSample();
        Sample s = history[HISTORY_SIZE - 1];
        Arrays.fill(history, s);
        historyCount = HISTORY_SIZE;
    }

    /**
     * Takes one detection sample and appends it to the history.
     *
     * WHY TWO DETECTION TYPES?
     *   The FTC SDK registers the 4 tags on each Hive face as a "cluster".
     *   When all 4 are visible, getDetections() returns an AprilTagClusterDetection
     *   identified by its cluster name (e.g. "RED_SCORING"), NOT by individual tag IDs.
     *   If only some tags are visible, individual AprilTagSingleDetection objects
     *   may be returned instead. This method handles both cases.
     *
     * HOW DETECTION WORKS:
     *   - If the non-audience cluster is visible → NON_AUDIENCE_HIGHER
     *   - If the audience cluster is visible     → AUDIENCE_HIGHER
     *   - If both visible, compare ftcPose.z     → higher z value = physically higher face
     *   - If nothing visible, use robot side as a fallback assumption
     *
     * NOTE ON ftcPose.z:
     *   ftcPose.z is the forward distance from the camera to the tag in inches.
     *   When the Hive is tipping, the upper face's tags are closer to a robot
     *   watching from the side, so they report a smaller (more negative) z.
     *   This comparison is only reliable when both clusters are simultaneously visible.
     *   For a fully tipped Hive, pure visibility is the primary signal.
     *
     * TODO: Calibrate a z threshold once you have field access if you want finer
     *       tipping detection. Log ftcPose.z values with addTelemetry() to find it.
     */
    private void takeSample() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        lastNonAudienceIds = new ArrayList<>();
        lastAudienceIds    = new ArrayList<>();
        lastClusterNames   = new ArrayList<>();

        double nonAudienceTotalZ = 0, audienceTotalZ = 0;
        int nonAudienceCount = 0, audienceCount = 0;

        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagSingleDetection) {
                // Individual tag — identify by numeric ID
                AprilTagSingleDetection d = (AprilTagSingleDetection) detection;
                double z = detection.ftcPose != null ? detection.ftcPose.z : 0;
                if (contains(nonAudienceIds, d.id)) {
                    lastNonAudienceIds.add(d.id);
                    nonAudienceTotalZ += z;
                    nonAudienceCount++;
                } else if (contains(audienceIds, d.id)) {
                    lastAudienceIds.add(d.id);
                    audienceTotalZ += z;
                    audienceCount++;
                }

            } else if (detection instanceof AprilTagClusterDetection) {
                // All 4 tags on a face detected together — identify by cluster name
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                String name = (cluster.metadata != null) ? cluster.metadata.name : "unknown";
                lastClusterNames.add(name);
                double z = detection.ftcPose != null ? detection.ftcPose.z : 0;
                String nameLower = name.toLowerCase();
                // Cluster names from the game set: "RED_SCORING"/"BLUE_SCORING" = non-audience
                //                                  "RED_AUDIENCE"/"BLUE_AUDIENCE" = audience
                boolean isAudience    = nameLower.contains("audience");
                boolean isNonAudience = !isAudience && nameLower.contains("scoring");
                if (isNonAudience) {
                    nonAudienceTotalZ += z;
                    nonAudienceCount++;
                } else if (isAudience) {
                    audienceTotalZ += z;
                    audienceCount++;
                }
            }
        }

        boolean sawNonAudience = nonAudienceCount > 0;
        boolean sawAudience    = audienceCount > 0;

        lastNonAudienceZ = sawNonAudience ? nonAudienceTotalZ / nonAudienceCount : Double.NaN;
        lastAudienceZ    = sawAudience    ? audienceTotalZ    / audienceCount    : Double.NaN;

        if (sawNonAudience && sawAudience) {
            // Both faces partially visible (mid-tip) — higher z = physically higher
            lastSample = (lastNonAudienceZ > lastAudienceZ)
                    ? Sample.NON_AUDIENCE_HIGHER
                    : Sample.AUDIENCE_HIGHER;
        } else if (sawNonAudience) {
            // Only non-audience tags visible → non-audience face is up
            lastSample = Sample.NON_AUDIENCE_HIGHER;
        } else if (sawAudience) {
            // Only audience tags visible → audience face is up
            lastSample = Sample.AUDIENCE_HIGHER;
        } else {
            // Nothing visible — fall back to robot-side assumption.
            // AUDIENCE side robot → non-audience face is facing us → assume non-audience up.
            // NON_AUDIENCE side robot → audience face is facing us → assume audience up.
            lastSample = (robotSide == RobotSide.AUDIENCE)
                    ? Sample.NON_AUDIENCE_HIGHER
                    : Sample.AUDIENCE_HIGHER;
        }

        // Shift history left and add the newest sample at the end
        System.arraycopy(history, 1, history, 0, HISTORY_SIZE - 1);
        history[HISTORY_SIZE - 1] = lastSample;
        if (historyCount < HISTORY_SIZE) historyCount++;
    }

    /**
     * Returns the current HiveState based on the rolling sample history.
     *
     * HISTORY LAYOUT (oldest → newest):
     *   history[0]  history[1]  history[2]  history[3]
     *   (oldest)                             (newest)
     *
     * RULES:
     *   All 4 NON_AUDIENCE_HIGHER → NON_AUDIENCE_UP   (stable)
     *   All 4 AUDIENCE_HIGHER     → AUDIENCE_UP        (stable)
     *   Old samples non-audience, recent samples audience → TIPPING_TO_AUDIENCE
     *   Old samples audience, recent samples non-audience → TIPPING_TO_NON_AUDIENCE
     *   Anything else → UNKNOWN
     *
     * Returns UNKNOWN until 4 samples have been collected (about 2 seconds).
     * Use initSample() at autonomous start to skip that warm-up period.
     */
    public HiveState getState() {
        if (historyCount < HISTORY_SIZE) return HiveState.UNKNOWN;

        boolean allNonAudience = true, allAudience = true;
        for (Sample s : history) {
            if (s != Sample.NON_AUDIENCE_HIGHER) allNonAudience = false;
            if (s != Sample.AUDIENCE_HIGHER)     allAudience    = false;
        }

        if (allNonAudience) return HiveState.NON_AUDIENCE_UP;
        if (allAudience)    return HiveState.AUDIENCE_UP;

        // Check for a clear directional transition in the history window
        boolean recentAudience    = history[2] == Sample.AUDIENCE_HIGHER     && history[3] == Sample.AUDIENCE_HIGHER;
        boolean recentNonAudience = history[2] == Sample.NON_AUDIENCE_HIGHER && history[3] == Sample.NON_AUDIENCE_HIGHER;
        boolean earlyNonAudience  = history[0] == Sample.NON_AUDIENCE_HIGHER && history[1] == Sample.NON_AUDIENCE_HIGHER;
        boolean earlyAudience     = history[0] == Sample.AUDIENCE_HIGHER     && history[1] == Sample.AUDIENCE_HIGHER;

        if (earlyNonAudience && recentAudience)    return HiveState.TIPPING_TO_AUDIENCE;
        if (earlyAudience    && recentNonAudience) return HiveState.TIPPING_TO_NON_AUDIENCE;

        // Catch a single-sample transition (history[3] just flipped)
        if (history[3] == Sample.AUDIENCE_HIGHER &&
                history[0] == Sample.NON_AUDIENCE_HIGHER &&
                history[1] == Sample.NON_AUDIENCE_HIGHER) return HiveState.TIPPING_TO_AUDIENCE;
        if (history[3] == Sample.NON_AUDIENCE_HIGHER &&
                history[0] == Sample.AUDIENCE_HIGHER &&
                history[1] == Sample.AUDIENCE_HIGHER) return HiveState.TIPPING_TO_NON_AUDIENCE;

        return HiveState.UNKNOWN;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- Hive Vision ---");
        telemetry.addData("Camera state", visionPortal.getCameraState());
        telemetry.addData("Raw detections", aprilTag.getDetections().size());

        // Cluster names — key diagnostic: shows what the SDK calls each cluster
        if (!lastClusterNames.isEmpty())
            telemetry.addData("Cluster names", lastClusterNames.toString());

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Robot side", robotSide);
        telemetry.addData("State", getState());

        // Non-audience group
        if (!lastNonAudienceIds.isEmpty())
            telemetry.addData("Non-audience single IDs", lastNonAudienceIds.toString());
        if (!Double.isNaN(lastNonAudienceZ))
            telemetry.addData("  ftcPose.z", "%.2f", lastNonAudienceZ);

        // Audience group
        if (!lastAudienceIds.isEmpty())
            telemetry.addData("Audience single IDs", lastAudienceIds.toString());
        if (!Double.isNaN(lastAudienceZ))
            telemetry.addData("  ftcPose.z", "%.2f", lastAudienceZ);

        // Height comparison when both visible
        if (!Double.isNaN(lastNonAudienceZ) && !Double.isNaN(lastAudienceZ)) {
            double diff = lastNonAudienceZ - lastAudienceZ;
            telemetry.addData("Height diff (z)", "%.2f  (%s higher)", Math.abs(diff),
                    diff > 0 ? "non-audience" : "audience");
        }

        // History
        for (int i = 0; i < history.length; i++) {
            String label = history[i] == Sample.NON_AUDIENCE_HIGHER ? "NON_AUDIENCE_HIGHER"
                         : history[i] == Sample.AUDIENCE_HIGHER     ? "AUDIENCE_HIGHER"
                         : "NO_DETECTION";
            telemetry.addData("History[" + i + "]", label);
        }
    }

    public void close() {
        visionPortal.close();
    }

    private static boolean contains(int[] arr, int val) {
        for (int v : arr) if (v == val) return true;
        return false;
    }
}
