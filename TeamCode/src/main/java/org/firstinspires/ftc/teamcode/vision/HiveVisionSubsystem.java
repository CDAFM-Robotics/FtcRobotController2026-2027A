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

public class HiveVisionSubsystem {

    public enum Alliance { RED, BLUE }
    public enum RobotSide { AUDIENCE, NON_AUDIENCE }

    private enum Sample { NON_AUDIENCE_HIGHER, AUDIENCE_HIGHER, NO_DETECTION }

    private static final int HISTORY_SIZE = 4;
    private static final double SAMPLE_INTERVAL_SEC = 0.5;

    private final Alliance alliance;
    private final RobotSide robotSide;
    private final int[] nonAudienceIds;
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

    public HiveVisionSubsystem(HardwareMap hardwareMap, Alliance alliance, RobotSide robotSide) {
        this.alliance = alliance;
        this.robotSide = robotSide;

        if (alliance == Alliance.RED) {
            nonAudienceIds = new int[]{30, 31, 32, 33};
            audienceIds    = new int[]{34, 35, 36, 37};
        } else {
            nonAudienceIds = new int[]{42, 43, 44, 45};
            audienceIds    = new int[]{38, 39, 40, 41};
        }

        Arrays.fill(history, Sample.NO_DETECTION);

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        sampleTimer.reset();
    }

    public void update() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return;
        if (sampleTimer.seconds() >= SAMPLE_INTERVAL_SEC) {
            sampleTimer.reset();
            takeSample();
        }
    }

    public void initSample() {
        takeSample();
        Sample s = history[HISTORY_SIZE - 1];
        Arrays.fill(history, s);
        historyCount = HISTORY_SIZE;
    }

    private void takeSample() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        lastNonAudienceIds = new ArrayList<>();
        lastAudienceIds    = new ArrayList<>();
        lastClusterNames   = new ArrayList<>();

        double nonAudienceTotalZ = 0, audienceTotalZ = 0;
        int nonAudienceCount = 0, audienceCount = 0;

        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagSingleDetection) {
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
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                String name = (cluster.metadata != null) ? cluster.metadata.name : "unknown";
                lastClusterNames.add(name);
                double z = detection.ftcPose != null ? detection.ftcPose.z : 0;
                String nameLower = name.toLowerCase();
                // "red scoring" / "blue scoring" = non-audience; "RED_AUDIENCE" / "BLUE_AUDIENCE" = audience
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
            // Higher ftcPose.z = physically higher
            lastSample = (lastNonAudienceZ > lastAudienceZ)
                    ? Sample.NON_AUDIENCE_HIGHER
                    : Sample.AUDIENCE_HIGHER;
        } else if (sawNonAudience) {
            lastSample = Sample.NON_AUDIENCE_HIGHER;
        } else if (sawAudience) {
            lastSample = Sample.AUDIENCE_HIGHER;
        } else {
            lastSample = (robotSide == RobotSide.AUDIENCE)
                    ? Sample.NON_AUDIENCE_HIGHER
                    : Sample.AUDIENCE_HIGHER;
        }

        System.arraycopy(history, 1, history, 0, HISTORY_SIZE - 1);
        history[HISTORY_SIZE - 1] = lastSample;
        if (historyCount < HISTORY_SIZE) historyCount++;
    }

    public HiveState getState() {
        if (historyCount < HISTORY_SIZE) return HiveState.UNKNOWN;

        boolean allNonAudience = true, allAudience = true;
        for (Sample s : history) {
            if (s != Sample.NON_AUDIENCE_HIGHER) allNonAudience = false;
            if (s != Sample.AUDIENCE_HIGHER)     allAudience    = false;
        }

        if (allNonAudience) return HiveState.NON_AUDIENCE_UP;
        if (allAudience)    return HiveState.AUDIENCE_UP;

        boolean recentAudience    = history[2] == Sample.AUDIENCE_HIGHER     && history[3] == Sample.AUDIENCE_HIGHER;
        boolean recentNonAudience = history[2] == Sample.NON_AUDIENCE_HIGHER && history[3] == Sample.NON_AUDIENCE_HIGHER;
        boolean earlyNonAudience  = history[0] == Sample.NON_AUDIENCE_HIGHER && history[1] == Sample.NON_AUDIENCE_HIGHER;
        boolean earlyAudience     = history[0] == Sample.AUDIENCE_HIGHER     && history[1] == Sample.AUDIENCE_HIGHER;

        if (earlyNonAudience && recentAudience)    return HiveState.TIPPING_TO_AUDIENCE;
        if (earlyAudience    && recentNonAudience) return HiveState.TIPPING_TO_NON_AUDIENCE;

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
