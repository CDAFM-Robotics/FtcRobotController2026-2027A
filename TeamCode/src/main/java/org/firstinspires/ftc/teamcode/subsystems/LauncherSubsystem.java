package org.firstinspires.ftc.teamcode.subsystems;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.devices.FlywheelMotorDevice;
import org.firstinspires.ftc.teamcode.util.LoopProfiler;

import java.util.function.BooleanSupplier;

@Configurable
public class LauncherSubsystem extends Subsystem {

    // Flywheel loop gains. These are pushed into the motor devices every loop, so Panels edits are live.
    // power = (kS + kV*target + kP*err + kI*integral) * (nominalVoltage / Vbat), clamped to [0, 1]. No kD on purpose:
    // hub velocity comes in 20 ticks/s steps and a derivative term only amplifies that.
    public static double kP = 0.002;
    public static double kI = 0.0001;
    public static double kIMax = 0.1;          // cap on the integral's contribution, in power units
    public static double kS = 0.0;             // static-friction power; measure on the robot (open-loop fit), 0 until then
    public static double kV = 1.0 / 2700.0;    // power per tick/s at nominalVoltage (was kFF)
    public static double nominalVoltage = 12.0; // battery compensation reference; 0 disables it
    public static int voltageReadPeriodMs = 250;
    public static int logEveryNLoops = 1;      // LauncherVelocity Log.d cadence; 0 = off

    private final FlywheelMotorDevice pollenLauncherMotor;
    private final FlywheelMotorDevice nectarLauncherMotor;

    private BindingManager bindingManager;

    private final Telemetry telemetry;
    private final TelemetryManager panelsTelemetry;

    public LauncherSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String pollenLauncherMotorName, String nectarLauncherMotorName) {
        pollenLauncherMotor = new FlywheelMotorDevice(hardwareMap, pollenLauncherMotorName);
        nectarLauncherMotor = new FlywheelMotorDevice(hardwareMap, nectarLauncherMotorName);

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
        // Software loop drives raw power: make sure the hub's own velocity PID is not cascaded underneath it.
        // FLOAT so a wheel at target 0 coasts instead of braking. Each is set once; nothing re-reads them later.
        pollenLauncherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pollenLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pollenLauncherMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        nectarLauncherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        nectarLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        nectarLauncherMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        applyGains();
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

    private long loopCount = 0;
    private final StringBuilder logLine = new StringBuilder(160);

    private void applyGains() {
        pollenLauncherMotor.setGains(kS, kV, kP, kI, kIMax).setNominalVoltage(nominalVoltage).setVoltageReadPeriodMs(voltageReadPeriodMs);
        nectarLauncherMotor.setGains(kS, kV, kP, kI, kIMax).setNominalVoltage(nominalVoltage).setVoltageReadPeriodMs(voltageReadPeriodMs);
    }

    @Override
    public void update() {
        applyGains();   // cheap field copies; this is what makes the Panels sliders live

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

        // Run the loops first so telemetry and the log describe this loop's command.
        nectarLauncherMotor.update();
        pollenLauncherMotor.update();

        double pollenActual = pollenLauncherMotor.getCurrentVelocity();
        double nectarActual = nectarLauncherMotor.getCurrentVelocity();

        telemetry.addData("Pollen Launcher Target Velocity", pollenLauncherVelocity);
        telemetry.addData("Nectar Launcher Target Velocity", nectarLauncherVelocity);
        telemetry.addLine();
        telemetry.addData("Pollen Launcher Actual Velocity", pollenActual);
        telemetry.addData("Nectar Launcher Actual Velocity", nectarActual);

        panelsTelemetry.addData("Pollen Target Velocity", pollenLauncherVelocity);
        panelsTelemetry.addData("Pollen Actual Velocity", pollenActual);
        panelsTelemetry.addData("Pollen Power", pollenLauncherMotor.getLastOutput());
        panelsTelemetry.addData("Nectar Target Velocity", nectarLauncherVelocity);
        panelsTelemetry.addData("Nectar Actual Velocity", nectarActual);
        panelsTelemetry.addData("Nectar Power", nectarLauncherMotor.getLastOutput());
        panelsTelemetry.addData("Battery V", pollenLauncherMotor.getBatteryVoltage());
        if (panelsTelemetry.getShouldUpdateLines()) panelsTelemetry.update(telemetry);

        // Per-loop record for offline analysis. Same keys as before (the log parsers rely on them) plus power and
        // battery voltage, which the Nov 5 analysis lacked. Built without String.format; cadence from Panels.
        loopCount++;
        if (logEveryNLoops > 0 && loopCount % logEveryNLoops == 0) {
            StringBuilder sb = logLine;
            sb.setLength(0);
            sb.append("t_ms=").append(System.nanoTime() / 1_000_000L)
              .append(" pollen_target=").append((int) pollenLauncherVelocity)
              .append(" pollen_actual=").append((int) pollenActual)
              .append(" nectar_target=").append((int) nectarLauncherVelocity)
              .append(" nectar_actual=").append((int) nectarActual)
              .append(" pollen_power=").append(Math.round(pollenLauncherMotor.getLastOutput() * 1000) / 1000.0)
              .append(" nectar_power=").append(Math.round(nectarLauncherMotor.getLastOutput() * 1000) / 1000.0)
              .append(" vbat=").append(Math.round(pollenLauncherMotor.getBatteryVoltage() * 100) / 100.0);
            Log.d("LauncherVelocity", sb.toString());
        }
        LoopProfiler.mark("launcher");   // includes the bulk read when this is the first subsystem
    }

    @Override
    public void stop() {


    }
}
