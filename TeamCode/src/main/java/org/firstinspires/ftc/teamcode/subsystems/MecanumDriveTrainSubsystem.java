package org.firstinspires.ftc.teamcode.subsystems;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.devices.actuators.MotorDevice;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

public class MecanumDriveTrainSubsystem extends Subsystem {

    private final MotorDevice frontLeftDriveMotor;
    private final MotorDevice backLeftDriveMotor;
    private final MotorDevice frontRightDriveMotor;
    private final MotorDevice backRightDriveMotor;

    private BindingManager bindingManager;

    public MecanumDriveTrainSubsystem(HardwareMap hardwareMap, String frontLeftName, String backLeftName, String frontRightName, String backRightName) {
        frontLeftDriveMotor = new MotorDevice(hardwareMap, frontLeftName);
        backLeftDriveMotor = new MotorDevice(hardwareMap, backLeftName);
        frontRightDriveMotor = new MotorDevice(hardwareMap, frontRightName);
        backRightDriveMotor = new MotorDevice(hardwareMap, backRightName);
    }

    public void setControls(BindingManager bindingManager, DoubleSupplier drive, DoubleSupplier strafe, DoubleSupplier turn, DoubleSupplier speed, DoubleSupplier heading) {
        bindingManager.addAnalog("drive", drive);
        bindingManager.addAnalog("strafe", strafe);
        bindingManager.addAnalog("turn", turn);
        bindingManager.addAnalog("driveSpeed", speed);
        bindingManager.addAnalog("heading", heading);
        this.bindingManager = bindingManager;
    }

    @Override
    public void init() {
        frontLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {



        double y = bindingManager.checkAnalog("drive");
        double x = bindingManager.checkAnalog("strafe");
        double rx = bindingManager.checkAnalog("turn");

        double heading = bindingManager.checkAnalog("heading");

        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        // put strafing factors here
        rotX = rotX * 1;
        rotY = rotY * 1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = ((rotY + rotX) + rx) / denominator;
        double backLeftPower = ((rotY - rotX) + rx) / denominator;
        double frontRightPower = ((rotY - rotX) - rx) / denominator;
        double backRightPower = ((rotY + rotX) - rx) / denominator;

        frontLeftDriveMotor.setPower(frontLeftPower);
        backLeftDriveMotor.setPower(backLeftPower);
        frontRightDriveMotor.setPower(frontRightPower);
        backRightDriveMotor.setPower(backRightPower);

        frontLeftDriveMotor.update();
        backLeftDriveMotor.update();
        frontRightDriveMotor.update();
        backRightDriveMotor.update();
    }

    @Override
    public void stop() {
        frontLeftDriveMotor.setPower(0).update();
        backLeftDriveMotor.setPower(0).update();
        frontRightDriveMotor.setPower(0).update();
        backRightDriveMotor.setPower(0).update();
    }
}
