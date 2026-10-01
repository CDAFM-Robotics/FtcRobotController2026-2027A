package org.firstinspires.ftc.teamcode.subsystems;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.devices.actuators.MotorDevice;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

public class IntakeSubsystem extends Subsystem {

    private final MotorDevice intakeMotor;

    private BindingManager bindingManager;

    public IntakeSubsystem(HardwareMap hardwareMap, String intakeMotorName) {
        intakeMotor = new MotorDevice(hardwareMap, intakeMotorName);
    }

    public void setBindings(BindingManager bindingManager, DoubleSupplier intakeSpeed) {
        bindingManager.addAnalog("intakeSpeed", intakeSpeed);

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
        intakeMotor.setPower(bindingManager.checkAnalog("intakeSpeed"));
        intakeMotor.update();
    }

    @Override
    public void stop() {

    }
}
