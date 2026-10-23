package org.firstinspires.ftc.teamcode.subsystems;

import com.cdarobotics.cdalib.devices.actuators.CRServoDevice;
import com.cdarobotics.cdalib.devices.actuators.MotorDevice;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.cdarobotics.cdalib.bindings.BindingManager;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TransferSubsystem extends Subsystem {

    private static final double TRANSFER_MOTOR_POWER = 1;
    private static final double TRANSFER_SERVO_POWER = 1;



    private final MotorDevice transferMotor;

    private final CRServoDevice leftTransferServo;
    private final CRServoDevice rightTransferServo;

    private BindingManager bindingManager;

    public TransferSubsystem(HardwareMap hardwareMap, String transferMotorName, String leftTransferServoName, String rightTransferServoName){
        transferMotor = new MotorDevice(hardwareMap, transferMotorName);
        leftTransferServo = new CRServoDevice(hardwareMap, leftTransferServoName);
        rightTransferServo = new CRServoDevice(hardwareMap, rightTransferServoName);
    }

    public void setBindings(BindingManager bindingManager, BooleanSupplier transferServo, BooleanSupplier transferMotor) {
        bindingManager.addBinding("transferServo", transferServo);
        bindingManager.addBinding("transferMotor", transferMotor);
        this.bindingManager = bindingManager;
    }

    @Override
    public void init() {
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightTransferServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

        if (bindingManager.checkBinding("transferMotor")) {
            transferMotor.setPower(TRANSFER_MOTOR_POWER);
        }
        else {
            transferMotor.setPower(0);
        }

        if (bindingManager.checkBinding("transferServo")) {
            leftTransferServo.setPower(TRANSFER_SERVO_POWER);
            rightTransferServo.setPower(TRANSFER_SERVO_POWER);
        }
        else {
            leftTransferServo.setPower(0);
            rightTransferServo.setPower(0);
        }

        transferMotor.update();

        leftTransferServo.update();
        rightTransferServo.update();
    }

    @Override
    public void stop() {

    }
}
