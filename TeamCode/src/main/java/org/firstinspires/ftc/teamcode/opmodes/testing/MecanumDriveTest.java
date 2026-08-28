package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrainSubsystem;

@TeleOp(name = "Mecanum Drive Test", group = "Testing")
public class MecanumDriveTest extends ModularOpMode {

    MecanumDriveTrainSubsystem md;

    @Override
    protected void preload() {
        md = new MecanumDriveTrainSubsystem(hardwareMap, "frontLeftDriveMotor", "backLeftDriveMotor", "frontRightDriveMotor", "backRightDriveMotor");

        md.setControls(bindingManager, () -> -gamepad1.left_stick_y, () -> gamepad1.left_stick_x, () -> gamepad1.right_stick_x, () -> 1, () -> 0); //TODO do something with speed and heading

        registerSubsystem(md);
    }
}
