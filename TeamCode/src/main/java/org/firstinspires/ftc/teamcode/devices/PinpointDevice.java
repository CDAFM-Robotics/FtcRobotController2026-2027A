package org.firstinspires.ftc.teamcode.devices;

import com.cdarobotics.cdalib.devices.Device;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class PinpointDevice extends Device {

    private final GoBildaPinpointDriver pinpoint;

    private final double pinpointYawScalar;

    public PinpointDevice(HardwareMap hardwareMap, String pinpointName, double pinpointOffsetX, double pinpointOffsetY, DistanceUnit distanceUnit) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, pinpointName);

        pinpoint.setOffsets(pinpointOffsetX, pinpointOffsetY, distanceUnit);

        pinpointYawScalar = pinpoint.getYawScalar();
    }

    @Override
    public void update() {
        pinpoint.update();

        if (pinpoint.getYawScalar() != pinpointYawScalar) {
            pinpoint.setYawScalar(pinpointYawScalar);
        }
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
