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

import java.util.Arrays;
import java.util.List;

public class HiveVisionSubsystemBasic {

    public enum BasicHiveStability {STABLE, TIPPING}
    public enum AdvancedHiveStability {STABLE, TIPPING_TO_AUDIENCE, TIPPING_TO_NON_AUDIENCE}
    public enum RobotSide { AUDIENCE, NON_AUDIENCE }
    private final RobotSide robotSide;
    private enum Sample { NON_AUDIENCE_HIGHER, AUDIENCE_HIGHER, NO_DETECTION, NOT_READABLE}
    public enum Alliance { RED, BLUE }
    public final Alliance alliance;
    private final int[] nonAudienceIds;
    private final int[] audienceIds;
    private final ElapsedTime sampleTimer = new ElapsedTime();


    //private List<String> HiveReadLocation1Red = new ArrayList<>();
    //private List<String> HiveReadLocation2Red = new ArrayList<>();
    //private List<String> HiveReadLocation1Blue = new ArrayList<>();
    //private List<String> HiveReadLocation2Blue = new ArrayList<>();

    public final double[] HiveReadlocationNonAudienceRed = {0.0, 0.0};
    public final double[] HiveReadLocationAudienceRed = {0.0, 0.0};
    public final double[] HiveReadLocationNonAudienceBlue = {0.0, 0.0};
    public final double[] HiveReadLocationAudienceBlue = {0.0, 0.0};
    public final double[] HiveReadLocationMiddleRed = {0.0, 0.0};
    public final double[] HiveReadLocationMiddleBlue = {0.0, 0.0};

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;
    private int HISTORY_SIZE = 4;
    private int historyCount = 0;

    public double SAMPLE_INTERVAL_SEC = 0.5;
    private double Pitch = 0.0;
    private final Sample[] history = new Sample[HISTORY_SIZE];
    private HiveVisionSubsystemBasic.Sample lastSample = HiveVisionSubsystemBasic.Sample.NO_DETECTION;
    private HiveVisionSubsystemBasic.BasicHiveStability basicHiveStability = HiveVisionSubsystemBasic.BasicHiveStability.STABLE;
    private HiveVisionSubsystemBasic.AdvancedHiveStability advancedHiveStability = HiveVisionSubsystemBasic.AdvancedHiveStability.STABLE;

    private double MinHivePitch = -45;
    private double MaxHivePitch = 45;
    private double HivePitchErrorMargin = 5;
    private double TotalHivePitch = 0; // a positive number would mean that the audience side is down
    private double LastTotalHivePitch = 0;

    public void initSample() {
        takeSample();
        HiveVisionSubsystemBasic.Sample s = history[HISTORY_SIZE - 1];
        Arrays.fill(history, s);
        historyCount = HISTORY_SIZE;
    }

    public void update() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return;
        if (sampleTimer.seconds() >= SAMPLE_INTERVAL_SEC) {
            sampleTimer.reset();
            takeSample();
        }
    }

    public HiveVisionSubsystemBasic(HardwareMap hardwareMap, Alliance alliance, RobotSide robotSide) {

        this.alliance = alliance;
        this.robotSide = robotSide;

        // Assign tag ID arrays based on alliance
        if (alliance == HiveVisionSubsystemBasic.Alliance.RED) {
            nonAudienceIds = new int[]{30, 31, 32, 33};
            audienceIds    = new int[]{34, 35, 36, 37};
        } else {
            nonAudienceIds = new int[]{42, 43, 44, 45};
            audienceIds    = new int[]{38, 39, 40, 41};
        }

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        Arrays.fill(history, HiveVisionSubsystemBasic.Sample.NO_DETECTION);

        sampleTimer.reset();
    }

//Right now Pitch==0 would mean that the hive is ballanced perfectly in the middle - may need to be revisited
    private void takeSample() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {

            Pitch = detection.ftcPose.pitch;
            AprilTagSingleDetection d = (AprilTagSingleDetection) detection;

            if (detection instanceof AprilTagSingleDetection) {
                if (contains(nonAudienceIds, d.id)) {
                    if (Pitch > 0) {
                        lastSample = Sample.NON_AUDIENCE_HIGHER;
                    }else if (Pitch == 0){
                        lastSample = Sample.NOT_READABLE;
                    }else if (Pitch < 0){
                        lastSample = Sample.AUDIENCE_HIGHER;
                    }
                    TotalHivePitch = -Pitch;
                } else if (contains(audienceIds, d.id)) {
                    if (Pitch > 0) {
                        lastSample = Sample.AUDIENCE_HIGHER;
                    }else if (Pitch == 0){
                        lastSample = Sample.NOT_READABLE;
                    }else if (Pitch < 0){
                        lastSample = Sample.NON_AUDIENCE_HIGHER;
                    }
                    TotalHivePitch = Pitch;
                }
            }
            else if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                String name = (cluster.metadata != null) ? cluster.metadata.name : "unknown";
                String nameLower = name.toLowerCase();
                boolean isAudience    = nameLower.contains("audience");
                boolean isNonAudience = !isAudience && nameLower.contains("scoring");
                if (isAudience) {
                    if (Pitch > 0) {
                        lastSample = Sample.NON_AUDIENCE_HIGHER;
                    }else if (Pitch == 0){
                        lastSample = Sample.NOT_READABLE;
                    }else if (Pitch < 0){
                        lastSample = Sample.AUDIENCE_HIGHER;
                    }
                    TotalHivePitch = Pitch;
                } else if (isNonAudience) {
                    if (Pitch > 0) {
                        lastSample = Sample.AUDIENCE_HIGHER;
                    }else if (Pitch == 0){
                        lastSample = Sample.NOT_READABLE;
                    }else if (Pitch < 0){
                        lastSample = Sample.NON_AUDIENCE_HIGHER;
                    }
                    TotalHivePitch = -Pitch;
                }
            }

            System.arraycopy(history, 1, history, 0, HISTORY_SIZE - 1);
            history[HISTORY_SIZE - 1] = lastSample;
            if (historyCount < HISTORY_SIZE) historyCount++;

            //Hive Stability - is the hive tipping?
            if (Pitch > (MaxHivePitch - HivePitchErrorMargin)|Pitch < (MinHivePitch + HivePitchErrorMargin)) {
                basicHiveStability = BasicHiveStability.STABLE;
            } else if (Pitch < (MaxHivePitch - HivePitchErrorMargin) & Pitch > (MinHivePitch + HivePitchErrorMargin)) {
                basicHiveStability = BasicHiveStability.TIPPING;
            }
        }

        if (TotalHivePitch < (LastTotalHivePitch + HivePitchErrorMargin)) {
            advancedHiveStability = AdvancedHiveStability.TIPPING_TO_NON_AUDIENCE;
        } else if (TotalHivePitch > (LastTotalHivePitch - HivePitchErrorMargin)) {
            advancedHiveStability = AdvancedHiveStability.TIPPING_TO_AUDIENCE;
        } else {
            advancedHiveStability = AdvancedHiveStability.STABLE;
        }
        LastTotalHivePitch = TotalHivePitch;

    }
//-------------------------------------telemetry is under construction------------------------------------------------------------
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("--- Hive Vision ---");
        telemetry.addData("Camera state", visionPortal.getCameraState());
        telemetry.addData("Raw detections", aprilTag.getDetections().size());

        // Cluster names — key diagnostic: shows what the SDK calls each cluster
//        if (!lastClusterNames.isEmpty())
//            telemetry.addData("Cluster names", lastClusterNames.toString());
//
//        telemetry.addData("Alliance", alliance);
//        telemetry.addData("Robot side", robotSide);
//        telemetry.addData("State", getState());
//
//        // Non-audience group
//        if (!lastNonAudienceIds.isEmpty())
//            telemetry.addData("Non-audience single IDs", lastNonAudienceIds.toString());
//        if (!Double.isNaN(lastNonAudienceZ))
//            telemetry.addData("  ftcPose.z", "%.2f", lastNonAudienceZ);
//
//        // Audience group
//        if (!lastAudienceIds.isEmpty())
//            telemetry.addData("Audience single IDs", lastAudienceIds.toString());
//        if (!Double.isNaN(lastAudienceZ))
//            telemetry.addData("  ftcPose.z", "%.2f", lastAudienceZ);
//
//        // Height comparison when both visible
//        if (!Double.isNaN(lastNonAudienceZ) && !Double.isNaN(lastAudienceZ)) {
//            double diff = lastNonAudienceZ - lastAudienceZ;
//            telemetry.addData("Height diff (z)", "%.2f  (%s higher)", Math.abs(diff),
//                    diff > 0 ? "non-audience" : "audience");
//        }

        // Pitch
        telemetry.addData("Hive Pitch", TotalHivePitch);
        // Hive State
        telemetry.addData("Basic Hive State", lastSample);
        telemetry.addData("Hive Motion", advancedHiveStability);
        // History
        for (int i = 0; i < history.length; i++) {
            String label = history[i] == HiveVisionSubsystemBasic.Sample.NON_AUDIENCE_HIGHER ? "NON_AUDIENCE_HIGHER"
                    : history[i] == HiveVisionSubsystemBasic.Sample.AUDIENCE_HIGHER     ? "AUDIENCE_HIGHER"
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
