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

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Configurable
public class LauncherSubsystem extends Subsystem {

    public static double kP  = 0.002;
    public static double kI  = 0.0001;
    public static double kD  = 0.0001;
    public static double kFF = 1.0 / 2700.0;

    private final VelocityMotorDevice pollenLauncherMotor;
    private final VelocityMotorDevice nectarLauncherMotor;

    private BindingManager bindingManager;

    private final Telemetry telemetry;
    private final TelemetryManager panelsTelemetry;

    public LauncherSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String pollenLauncherMotorName, String nectarLauncherMotorName) {
        pollenLauncherMotor = new VelocityMotorDevice(hardwareMap, pollenLauncherMotorName, new CompoundController(0, new PIDController(0, kP, kI, kD), new FFController(0, 0, kFF, 0)));
        nectarLauncherMotor = new VelocityMotorDevice(hardwareMap, nectarLauncherMotorName, new CompoundController(0, new PIDController(0, kP, kI, kD), new FFController(0, 0, kFF, 0)));

        this.telemetry = telemetry;
        this.panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
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

    private double pollenLauncherVelocity = 1460; // ~3321 RPM (ticks/sec = RPM / 60 * 28)
    private double nectarLauncherVelocity = 1540; // ~3900 RPM (ticks/sec = RPM / 60 * 28)

    public double getPollenTargetVelocity() { return pollenLauncherVelocity; }
    public double getPollenCurrentVelocity() { return pollenLauncherMotor.getCurrentVelocity(); }
    public double getNectarTargetVelocity() { return nectarLauncherVelocity; }
    public double getNectarCurrentVelocity() { return nectarLauncherMotor.getCurrentVelocity(); }
    
    private final double LAUNCHER_MOTOR_VELOCITY_TUNING = 20;

    private boolean prevIncreasePollenVelocity = false;
    private boolean prevDecreasePollenVelocity = false;
    private boolean prevIncreaseNectarVelocity = false;
    private boolean prevDecreaseNectarVelocity = false;

    @Override
    public void update() {
        if (bindingManager.checkBinding("pollenMotor")) {
            pollenLauncherMotor.setVelocity(pollenLauncherVelocity);
        } else {
            pollenLauncherMotor.setVelocity(0);
        }

        if (bindingManager.checkBinding("nectarMotor")) {
            nectarLauncherMotor.setVelocity(nectarLauncherVelocity);
        } else {
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

        panelsTelemetry.addData("Pollen Target Velocity", pollenLauncherVelocity);
        panelsTelemetry.addData("Pollen Actual Velocity", pollenLauncherMotor.getCurrentVelocity());
        panelsTelemetry.addData("Nectar Target Velocity", nectarLauncherVelocity);
        panelsTelemetry.addData("Nectar Actual Velocity", nectarLauncherMotor.getCurrentVelocity());
        if (panelsTelemetry.getShouldUpdateLines()) panelsTelemetry.update(telemetry);

        Log.d("LauncherVelocity", String.format("pollen_target=%.1f pollen_actual=%.1f nectar_target=%.1f nectar_actual=%.1f",
                pollenLauncherVelocity, pollenLauncherMotor.getCurrentVelocity(),
                nectarLauncherVelocity, nectarLauncherMotor.getCurrentVelocity()));

        nectarLauncherMotor.update();
        pollenLauncherMotor.update();
    }

    @Override
    public void stop() {


    }
}
