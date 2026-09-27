package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

@TeleOp(name = "Launcher Subsystem Test", group = "Testing")
public class LauncherSubsystemTest extends ModularOpMode {

    private LauncherSubsystem launcherSubsystem;

    @Override
    protected void preload() {
        launcherSubsystem = new LauncherSubsystem(hardwareMap, "pollenLauncherMotor", "nectarLauncherMotor", "launcherCRServo", telemetry);

        launcherSubsystem.setBindings(bindingManager, () -> true, () -> false);

        registerSubsystem(launcherSubsystem);
    }

    @Override
    public void loop() {
        super.loop();

        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}