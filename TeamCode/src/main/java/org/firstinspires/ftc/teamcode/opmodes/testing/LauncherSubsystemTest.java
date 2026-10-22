package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

@TeleOp (name = "Launcher Subsystem Test", group = "Testing")
@Photon
public class LauncherSubsystemTest extends ModularOpMode {

    private LauncherSubsystem launcherSubsystem;

    @Override
    protected void preload() {
        launcherSubsystem = new LauncherSubsystem(hardwareMap, telemetry, "pollenLauncherMotor", "nectarLauncherMotor");

        launcherSubsystem.setBindings(bindingManager, () -> gamepad1.right_bumper, () -> gamepad1.left_bumper, () -> gamepad1.dpadUpWasPressed(), () -> gamepad1.dpadDownWasPressed(), () -> gamepad1.dpadRightWasPressed(), () -> gamepad1.dpadLeftWasPressed());

        registerSubsystem(launcherSubsystem);
    }

    @Override
    public void loop() {
        super.loop();

        telemetry.update();
    }
}
