package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.DoubleSupplier;

@TeleOp(name = "Intake Test", group = "Testing")
@Photon
public class IntakeSubsystemTest extends ModularOpMode {

    private IntakeSubsystem intakeSubsystem;

    @Override
    protected void preload() {
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, "intakeMotor", "leftExtenderServo", "rightExtenderServo");

        intakeSubsystem.setBindings(bindingManager, () -> gamepad1.right_bumper, () -> gamepad1.left_bumper);

        registerSubsystem(intakeSubsystem);
    }
}
