package org.firstinspires.ftc.teamcode.subsystems;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.controllers.CompoundController;
import com.cdarobotics.cdalib.controllers.FFController;
import com.cdarobotics.cdalib.controllers.PIDController;
import com.cdarobotics.cdalib.devices.actuators.CRServoDevice;
import com.cdarobotics.cdalib.devices.actuators.MotorDevice;
import com.cdarobotics.cdalib.devices.actuators.ServoDevice;
import com.cdarobotics.cdalib.devices.actuators.VelocityMotorDevice;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class LauncherSubsystem extends Subsystem {

    private final VelocityMotorDevice pollenLauncherMotor;
    private final VelocityMotorDevice nectarLauncherMotor;

    private BindingManager bindingManager;

    private final Telemetry telemetry;

    public LauncherSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String pollenLauncherMotorName, String nectarLauncherMotorName) {
        pollenLauncherMotor = new VelocityMotorDevice(hardwareMap, pollenLauncherMotorName, new CompoundController(0, new PIDController(0, 0.001, 0, 0), new FFController(0, 0, 1/2700.0, 0)));
        nectarLauncherMotor = new VelocityMotorDevice(hardwareMap, nectarLauncherMotorName, new CompoundController(0, new PIDController(0, 0.001, 0, 0), new FFController(0, 0, 1/2700.0, 0)));

        this.telemetry = telemetry;
    }
    public void setBindings(BindingManager bindingManager, BooleanSupplier pollenMotor, BooleanSupplier nectarMotor, BooleanSupplier increasePollenMotorVelocity, BooleanSupplier decreasePollenMotorVelocity, BooleanSupplier increaseNectarMotorVelocity, BooleanSupplier decreaseNectarMotorVelocity) {
        bindingManager.addBinding("pollenMotor", pollenMotor);
        bindingManager.addBinding("nectarMotor", nectarMotor);

        bindingManager.addBinding("increasePollenMotorVelocity", increasePollenMotorVelocity);
        bindingManager.addBinding("decreasePollenMotorVelocity", decreasePollenMotorVelocity);
        bindingManager.addBinding("increaseNectarMotorVelocity", increaseNectarMotorVelocity);
        bindingManager.addBinding("decreaseNectarMotorVelocity", decreaseNectarMotorVelocity);

        this.bindingManager = bindingManager;
    }

    public void setBindings(BindingManager bindingManager, BooleanSupplier pollenMotor, BooleanSupplier nectarMotor) {
        bindingManager.addBinding("pollenMotor", pollenMotor);
        bindingManager.addBinding("nectarMotor", nectarMotor);

        bindingManager.addBinding("increasePollenMotorVelocity", () -> false);
        bindingManager.addBinding("decreasePollenMotorVelocity", () -> false);
        bindingManager.addBinding("increaseNectarMotorVelocity", () -> false);
        bindingManager.addBinding("decreaseNectarMotorVelocity", () -> false);

        this.bindingManager = bindingManager;
    }

    @Override
    public void init() {
        // pollenLauncherMotor
        pollenLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pollenLauncherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        pollenLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // nectarLauncherMotor
        nectarLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        nectarLauncherMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        nectarLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    private double pollenLauncherVelocity = 1550; // ~3321 RPM (ticks/sec = RPM / 60 * 28)
    private double nectarLauncherVelocity = 1820; // ~3900 RPM (ticks/sec = RPM / 60 * 28)

    public double getPollenTargetVelocity() { return pollenLauncherVelocity; }
    public double getPollenCurrentVelocity() { return pollenLauncherMotor.getCurrentVelocity(); }
    public double getNectarTargetVelocity() { return nectarLauncherVelocity; }
    public double getNectarCurrentVelocity() { return nectarLauncherMotor.getCurrentVelocity(); }
    
    private final double LAUNCHER_MOTOR_VELOCITY_TUNING = 50;

    private boolean prevIncreasePollenVelocity = false;
    private boolean prevDecreasePollenVelocity = false;
    private boolean prevIncreaseNectarVelocity = false;
    private boolean prevDecreaseNectarVelocity = false;

    @Override
    public void update() {
        if (bindingManager.checkBinding("pollenMotor")) {
            pollenLauncherMotor.setVelocity(pollenLauncherVelocity);
        }
        else {
            pollenLauncherMotor.setVelocity(0);
        }

        if (bindingManager.checkBinding("nectarMotor")) {
            nectarLauncherMotor.setVelocity(nectarLauncherVelocity);
        }
        else {
            nectarLauncherMotor.setVelocity(0);
        }

        boolean increasePollenNow = bindingManager.checkBinding("increasePollenMotorVelocity");
        boolean decreasePollenNow = bindingManager.checkBinding("decreasePollenMotorVelocity");
        boolean increaseNectarNow = bindingManager.checkBinding("increaseNectarMotorVelocity");
        boolean decreaseNectarNow = bindingManager.checkBinding("decreaseNectarMotorVelocity");

        if (increasePollenNow && !prevIncreasePollenVelocity) pollenLauncherVelocity += LAUNCHER_MOTOR_VELOCITY_TUNING;
        if (decreasePollenNow && !prevDecreasePollenVelocity) pollenLauncherVelocity -= LAUNCHER_MOTOR_VELOCITY_TUNING;
        if (increaseNectarNow && !prevIncreaseNectarVelocity) nectarLauncherVelocity += LAUNCHER_MOTOR_VELOCITY_TUNING;
        if (decreaseNectarNow && !prevDecreaseNectarVelocity) nectarLauncherVelocity -= LAUNCHER_MOTOR_VELOCITY_TUNING;

        prevIncreasePollenVelocity = increasePollenNow;
        prevDecreasePollenVelocity = decreasePollenNow;
        prevIncreaseNectarVelocity = increaseNectarNow;
        prevDecreaseNectarVelocity = decreaseNectarNow;

        telemetry.addData("Pollen Launcher Target Velocity", pollenLauncherVelocity);
        telemetry.addData("Nectar Launcher Target Velocity", nectarLauncherVelocity);
        telemetry.addLine();
        telemetry.addData("Pollen Launcher Actual Velocity", pollenLauncherMotor.getCurrentVelocity());
        telemetry.addData("Nectar Launcher Actual Velocity", nectarLauncherMotor.getCurrentVelocity());


        nectarLauncherMotor.update();
        pollenLauncherMotor.update();
    }

    @Override
    public void stop() {


    }
}
