package org.firstinspires.ftc.teamcode.subsystems;

import com.cdarobotics.cdalib.devices.actuators.MotorDevice;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.cdarobotics.cdalib.bindings.BindingManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

public class TransferSubsystem extends Subsystem {

    private final MotorDevice transferMotor;

    private BindingManager bindingManager;

    public TransferSubsystem(HardwareMap hardwareMap, String transferMotorName){
        transferMotor = new MotorDevice(hardwareMap, transferMotorName);
    }

    public void setBindings(BindingManager bindingManager){

        this.bindingManager = bindingManager;
    }

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        transferMotor.setPower(bindingManager.checkAnalog("intakeSpeed"));
        transferMotor.update();
    }

    @Override
    public void stop() {

    }
}
