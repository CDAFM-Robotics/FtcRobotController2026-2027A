package org.firstinspires.ftc.teamcode.subsystems;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.devices.actuators.CRServoDevice;
import com.cdarobotics.cdalib.devices.actuators.MotorDevice;
import com.cdarobotics.cdalib.devices.actuators.ServoDevice;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class LauncherSubsystem extends Subsystem {

    private final MotorDevice pollenLauncherMotor;
    // private final MotorDevice nectarLauncherMotor;

    // private final CRServoDevice launcherCRServo;

    private BindingManager bindingManager;

    public LauncherSubsystem(HardwareMap hardwareMap, String pollenLauncherMotorName, String nectarLauncherMotorName, String launcherCRServoName) {
        pollenLauncherMotor = new MotorDevice(hardwareMap, pollenLauncherMotorName);
        // nectarLauncherMotor = new MotorDevice(hardwareMap, nectarLauncherMotorName);

    }

    public void setBindings(BindingManager bindingManager, BooleanSupplier pollenMotor, BooleanSupplier nectarMotor) {
        bindingManager.addBinding("pollenMotor", pollenMotor);
        bindingManager.addBinding("nectarMotor", nectarMotor);

        this.bindingManager = bindingManager;
    }

    @Override
    public void init() {
        pollenLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pollenLauncherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // nectarLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT)

        // launcherCRServo = new CRServoDevice(hardwareMap, launcherCRServoName);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        if (bindingManager.checkBinding("pollenMotor")) {
            pollenLauncherMotor.setPower(0.5);
        }
        else {
            pollenLauncherMotor.setPower(0);
        }
        pollenLauncherMotor.update();
    }

    @Override
    public void stop() {

    }
}
