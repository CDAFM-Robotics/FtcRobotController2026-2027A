package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.api.Paths.line;

import android.annotation.SuppressLint;

import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.devices.LimelightDevice;
import org.firstinspires.ftc.teamcode.util.Vector2D;

import java.util.List;

public class LimelightDetectorSubsystem extends Subsystem {

    public static final double LIMELIGHT_MOUNT_ANGLE = 0;
    public static final double LIMELIGHT_MOUNT_HEIGHT = 7;

    public enum LimelightPipeline {
        POLLEN_DETECTOR,
        PIPELINE_1,
        PIPELINE_2,
        PIPELINE_3,
        PIPELINE_4,
        PIPELINE_5,
        PIPELINE_6,
        PIPELINE_7
    }

    private LimelightDevice limelight;
    private Telemetry telemetry;

    public LimelightDetectorSubsystem(Limelight3A limelight, Telemetry telemetry) {
        this.limelight = new LimelightDevice(limelight);
        this.telemetry = telemetry;
    }


    @Override
    public void init() {
        limelight.setPipeline(0);
        limelight.start();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    private Vector2D targetBall = new Vector2D(0, 0);

    @SuppressLint("DefaultLocale")
    @Override
    public void update() {
        limelight.update();

        List<LLResultTypes.DetectorResult> detectorResults = limelight.getDetectorResults();

        if (!detectorResults.isEmpty()) {

            Vector2D[] pollenLocations = new Vector2D[detectorResults.toArray().length];

            for (int i = 0; i < detectorResults.toArray().length; i++) {
                LLResultTypes.DetectorResult detectorResult = detectorResults.get(i);
                double tx = Math.toRadians(detectorResult.getTargetXDegrees());
                double ty = Math.toRadians(detectorResult.getTargetYDegrees());

                double distance = Math.tan(Math.PI / 2 - LIMELIGHT_MOUNT_ANGLE + ty) * (LIMELIGHT_MOUNT_HEIGHT - 1.4);

                pollenLocations[i] = new Vector2D(-Math.sin(tx) * distance, Math.cos(tx) * distance);

                telemetry.addLine(String.format("Ball: %d, tx: %.2f, ty: %.2f, distance: %.2f, x pos: %.2f, y pos: %.2f", i, tx, ty, distance, pollenLocations[i].getX(), pollenLocations[i].getY()));
                telemetry.addLine();
            }

            int greedyIndex = 0;
            double greedyDistance = Double.MAX_VALUE;


            for (int i = 0; i < pollenLocations.length; i++) {
                if (greedyDistance > pollenLocations[i].getDist()) {
                    greedyDistance = pollenLocations[i].getDist();
                    greedyIndex = i;
                }
            }

            targetBall = pollenLocations[greedyIndex];

            telemetry.addLine(String.format("Target Ball: %d, distance: %.2f", greedyIndex, greedyDistance));
        }
        else {
            telemetry.addLine("No Results");
        }
    }

    public Vector2D getTargetBall() {
        return targetBall;
    }

    public Path getPathToTargetBall(Pose currentPose) {

        double deltaX = targetBall.getX() * Math.cos(currentPose.heading()) - targetBall.getY() * Math.sin(currentPose.heading());
        double deltaY = targetBall.getX() * Math.sin(currentPose.heading()) + targetBall.getY() * Math.cos(currentPose.heading());

        Pose targetBallField = new Pose(currentPose.x() + deltaX, currentPose.y() + deltaY);

        return  line(currentPose, targetBallField)
            .linear(currentPose.heading(), currentPose.heading() + targetBall.getAngle());
    }

    @Override
    public void stop() {

    }
}
