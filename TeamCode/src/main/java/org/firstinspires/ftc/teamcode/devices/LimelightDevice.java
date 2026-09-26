package org.firstinspires.ftc.teamcode.devices;

import com.cdarobotics.cdalib.devices.Device;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

public class LimelightDevice extends Device {

    private Limelight3A limelight;
    private LLResult latestResult;

    public LimelightDevice(Limelight3A limelight) {
        this.limelight = limelight;
    }

    @Override
    public void update() {
        latestResult = limelight.getLatestResult();
    }

    public void start() {
        limelight.start();
    }

    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public LLResult getLimelightResult() {
        return latestResult;
    }

    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        return latestResult.getFiducialResults();
    }

    public List<LLResultTypes.BarcodeResult> getBarcodeResults() {
        return latestResult.getBarcodeResults();
    }

    public List<LLResultTypes.ClassifierResult> getClassifierResults() {
        return latestResult.getClassifierResults();
    }

    public List<LLResultTypes.ColorResult> getColorResults() {
        return latestResult.getColorResults();
    }

    public List<LLResultTypes.DetectorResult> getDetectorResults() {
        return latestResult.getDetectorResults();
    }
}