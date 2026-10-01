package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.opmodes.ModularOpMode;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeSubsystemTest extends ModularOpMode {

    private IntakeSubsystem intakeSubsystem;

    @Override
    protected void preload() {
        intakeSubsystem = new IntakeSubsystem(hardwareMap, "intakeMotor");

        intakeSubsystem.setBindings(bindingManager, () -> gamepad1.right_bumper ? 1 : 0);

        registerSubsystem(intakeSubsystem);
    }
}
