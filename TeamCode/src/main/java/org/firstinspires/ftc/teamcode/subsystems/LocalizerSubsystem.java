package org.firstinspires.ftc.teamcode.subsystems;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.util.LoopProfiler;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.devices.PinpointDevice;

public class LocalizerSubsystem extends Subsystem {

    private final PinpointDevice pinpoint;

    private BindingManager bindingManager;

    // Pod offsets, pod type and encoder directions all come from Pedro's Constants.localizerConfig, so TeleOp and
    // autonomous configure the Pinpoint identically. Re-run the Pinpoint Tuner and paste its block into Constants.java
    // to change them. (Nov 16: this used its own 13.25 / -9.75 cm offsets and never set the directions, so TeleOp
    // depended on whatever the previous OpMode had left on the chip.)
    public LocalizerSubsystem(HardwareMap hardwareMap, String pinpointName) {
        pinpoint = new PinpointDevice(hardwareMap, pinpointName, Constants.localizerConfig);
    }

    public void setBindings(BindingManager bindingManager) {
        bindingManager.addAnalog("heading", pinpoint::getHeading);
    }

    /** X position in inches */
    public double getX() { return pinpoint.getX(); }

    /** Y position in inches */
    public double getY() { return pinpoint.getY(); }

    /** Heading in radians */
    public double getHeading() { return pinpoint.getHeading(); }

    @Override
    public void init() {
        pinpoint.update();
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
        LoopProfiler.mark("localizer");
    }

    @Override
    public void stop() {

    }
}
