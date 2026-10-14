package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;
@TeleOp(name = "Intake Test", group = "Testing")

public class IntakeSubsystemTest extends ModularOpMode {

    private IntakeSubsystem intakeSubsystem;

    @Override
    protected void preload() {
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, "intakeMotor", "extenderServo1", "extenderServo2");

        intakeSubsystem.setBindings(bindingManager, () -> gamepad1.right_bumper ? 1 : 0, () -> gamepad1.left_bumper);

        registerSubsystem(intakeSubsystem);
    }

    @Override
    public void loop() {
        super.loop();
        telemetry.update();
    }
}
