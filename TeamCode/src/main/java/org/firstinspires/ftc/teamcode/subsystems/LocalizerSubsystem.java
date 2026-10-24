package org.firstinspires.ftc.teamcode.subsystems;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.devices.PinpointDevice;

public class LocalizerSubsystem extends Subsystem {

    private final PinpointDevice pinpoint;

    private BindingManager bindingManager;

    public static final double DEFAULT_PINPOINT_OFFSET_X_CM = 13.25;
    public static final double DEFAULT_PINPOINT_OFFSET_Y_CM = -9.75;

    public LocalizerSubsystem(HardwareMap hardwareMap, String pinpointName, double pinpointOffsetX, double pinpointOffsetY, DistanceUnit distanceUnit) {
        pinpoint = new PinpointDevice(hardwareMap, pinpointName, pinpointOffsetX, pinpointOffsetY, distanceUnit);
    }

    public void setBindings(BindingManager bindingManager) {
        bindingManager.addAnalog("heading", pinpoint::getHeading);
    }

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        pinpoint.update();
    }

    @Override
    public void stop() {

    }
}
