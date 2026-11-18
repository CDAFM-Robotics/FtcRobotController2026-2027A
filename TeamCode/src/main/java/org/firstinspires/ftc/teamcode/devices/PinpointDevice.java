package org.firstinspires.ftc.teamcode.devices;

import com.cdarobotics.cdalib.devices.Device;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class PinpointDevice extends Device {

    private final GoBildaPinpointDriver pinpoint;

    private final double pinpointYawScalar;

    /**
     * Configures the Pinpoint exactly as Pedro's PinpointLocalizer does from the same config: offsets, encoder
     * resolution (ticksPerUnit if set, otherwise the pod type) and both encoder directions. Every setting is written,
     * so nothing depends on what an earlier OpMode left on the chip. Does not reset the position.
     */
    public PinpointDevice(HardwareMap hardwareMap, String pinpointName, PinpointConfig config) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);

        pinpoint.setOffsets(config.xPodOffset.get(), config.yPodOffset.get(), config.offsetUnits.get());
        if (config.ticksPerUnit.get().isPresent()) {
            pinpoint.setEncoderResolution(config.ticksPerUnit.get().getAsDouble(), config.encoderResolutionUnit.get());
        } else {
            pinpoint.setEncoderResolution(config.podType.get());
        }
        pinpoint.setEncoderDirections(config.xPodDirection.get(), config.yPodDirection.get());

        pinpointYawScalar = pinpoint.getYawScalar();
    }

    @Override
    public void update() {
        pinpoint.update();

        if (pinpoint.getYawScalar() != pinpointYawScalar) {
            pinpoint.setYawScalar(pinpointYawScalar);
        }
    }

    /** Overwrites the Pinpoint's position and heading (inches, radians). The robot should be still. */
    public void setPose(double xInches, double yInches, double headingRadians) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xInches, yInches, AngleUnit.RADIANS, headingRadians));
    }

    public double getHeading() {
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }

    public double getX() {
        return pinpoint.getPosX(DistanceUnit.INCH);
    }

    public double getY() {
        return pinpoint.getPosY(DistanceUnit.INCH);
    }

    public double getHeadingVelocity() {
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    public double getXVelocity() {
        return pinpoint.getVelX(DistanceUnit.INCH);
    }

    public double getYVelocity() {
        return pinpoint.getVelY(DistanceUnit.INCH);
    }


}
