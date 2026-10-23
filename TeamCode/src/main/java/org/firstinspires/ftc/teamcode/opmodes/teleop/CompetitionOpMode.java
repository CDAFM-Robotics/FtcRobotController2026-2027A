package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

@TeleOp(name = "Competition TeleOp", group = "0Competition")
public class CompetitionOpMode extends ModularOpMode {

    private MecanumDriveTrainSubsystem mecanumDriveTrainSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private TransferSubsystem transferSubsystem;
    private LauncherSubsystem launcherSubsystem;

    @Override
    protected void preload() {
        mecanumDriveTrainSubsystem = new MecanumDriveTrainSubsystem(hardwareMap, "frontLeftDriveMotor", "backLeftDriveMotor", "frontRightDriveMotor", "backRightDriveMotor");

        mecanumDriveTrainSubsystem.setBindings(bindingManager, () -> -gamepad1.left_stick_y, () -> gamepad1.left_stick_x, () -> gamepad1.right_stick_x, () -> 1, () -> 0);

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, "intakeMotor", "leftExtenderServo", "rightExtenderServo");

        intakeSubsystem.setBindings(bindingManager, () -> gamepad1.right_bumper, () -> gamepad1.circle);

        transferSubsystem = new TransferSubsystem(hardwareMap, "transferMotor", "leftTransferServo", "rightTransferServo");

        transferSubsystem.setBindings(bindingManager, () -> gamepad1.right_bumper || gamepad2.circle, () -> gamepad2.circle);

        launcherSubsystem = new LauncherSubsystem(hardwareMap, telemetry, "pollenLauncherMotor", "nectarLauncherMotor");

        launcherSubsystem.setBindings(bindingManager, () -> gamepad2.right_bumper, () -> gamepad2.left_bumper);

        registerSubsystem(mecanumDriveTrainSubsystem);
        registerSubsystem(intakeSubsystem);
        registerSubsystem(transferSubsystem);
        registerSubsystem(launcherSubsystem);
    }
}
