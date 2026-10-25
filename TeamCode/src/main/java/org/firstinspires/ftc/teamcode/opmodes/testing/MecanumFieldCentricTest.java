package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrainSubsystem;

@TeleOp(name = "Mecanum Field Centric", group = "Testing")
public class MecanumFieldCentricTest extends ModularOpMode {

    private LocalizerSubsystem localizerSubsystem;
    private MecanumDriveTrainSubsystem mecanumDriveTrainSubsystem;

    @Override
    protected void preload() {
        localizerSubsystem = new LocalizerSubsystem(hardwareMap, "pinpoint", 13.25, -9.75f, DistanceUnit.CM);
        localizerSubsystem.setBindings(bindingManager);

        mecanumDriveTrainSubsystem = new MecanumDriveTrainSubsystem(hardwareMap, "frontLeftDriveMotor", "backLeftDriveMotor", "frontRightDriveMotor", "backRightDriveMotor");
        mecanumDriveTrainSubsystem.setBindings(bindingManager, () -> -gamepad1.left_stick_y, () -> gamepad1.left_stick_x, () -> gamepad1.right_stick_x, () -> 1, () -> bindingManager.checkAnalog("heading"));

        registerSubsystem(localizerSubsystem);
        registerSubsystem(mecanumDriveTrainSubsystem);
    }
}
