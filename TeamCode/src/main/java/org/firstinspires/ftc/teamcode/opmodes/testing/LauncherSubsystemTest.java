package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;

public class LauncherSubsystemTest extends ModularOpMode {

    private LauncherSubsystem launcherSubsystem;

    @Override
    protected void preload() {
        launcherSubsystem = new LauncherSubsystem(hardwareMap, "pollenLauncherMotor", "nectarLauncherMotor", "launcherCRServo");

        launcherSubsystem.setBindings(bindingManager, () -> gamepad1.right_bumper, () -> false);

        registerSubsystem(launcherSubsystem);
    }
}
