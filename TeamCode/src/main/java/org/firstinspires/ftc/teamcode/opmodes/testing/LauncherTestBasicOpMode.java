package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Launcher Test Basic Op Mode", group = "Testing")
public class LauncherTestBasicOpMode extends OpMode {

    DcMotorEx motor;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "pollenLauncherMotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        motor.setPower(0.5);
    }
}
