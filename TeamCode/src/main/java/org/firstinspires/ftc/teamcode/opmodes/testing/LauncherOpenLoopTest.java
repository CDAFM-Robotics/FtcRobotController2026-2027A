package org.firstinspires.ftc.teamcode.opmodes.testing;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Open-loop characterisation of both launcher flywheels, for kS and kV (flywheel loop review, finding 7).
 *
 * <p>On start it steps both motors through powerStart .. powerMax in powerStep increments, holding each level for
 * holdSeconds, then stops itself. Every loop logs {@code LauncherOpenLoop: t_ms step power pollen_vel nectar_vel
 * vbat settled}. Fit, per wheel, over the settled samples: {@code power * vbat / 12 = kS + kV * velocity}. No gamepad
 * input is used. Needs nothing in the transfer: anything there will be launched.
 */
@Configurable
@TeleOp(name = "Launcher Open Loop Test", group = "Testing")
public class LauncherOpenLoopTest extends OpMode {

    public static double powerStart = 0.2;
    public static double powerStep = 0.1;
    public static double powerMax = 0.8;
    public static double holdSeconds = 2.5;     // per level
    public static double settleSeconds = 1.2;   // last part of each hold counted as steady state
    public static double nominalVoltage = 12.0;

    private DcMotorEx pollen;
    private DcMotorEx nectar;
    private VoltageSensor battery;
    private long startNs;
    private int steps;
    private boolean done = false;
    private final StringBuilder sb = new StringBuilder(160);

    @Override
    public void init() {
        pollen = hardwareMap.get(DcMotorEx.class, "pollenLauncherMotor");
        nectar = hardwareMap.get(DcMotorEx.class, "nectarLauncherMotor");
        for (DcMotorEx m : new DcMotorEx[]{pollen, nectar}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setDirection(DcMotorSimple.Direction.FORWARD);
            m.setPower(0);
        }
        battery = hardwareMap.voltageSensor.iterator().next();
        steps = (int) Math.floor((powerMax - powerStart) / powerStep + 1e-9) + 1;
        telemetry.addData("plan", "%d levels, %.1f to %.1f, %.1f s each", steps, powerStart, powerMax, holdSeconds);
    }

    @Override
    public void start() {
        startNs = System.nanoTime();
    }

    @Override
    public void loop() {
        double t = (System.nanoTime() - startNs) * 1e-9;
        int step = (int) (t / holdSeconds);
        if (step >= steps || done) {
            pollen.setPower(0);
            nectar.setPower(0);
            if (!done) {
                done = true;
                Log.d("LauncherOpenLoop", "done");
                requestOpModeStop();
            }
            return;
        }
        double power = powerStart + step * powerStep;
        pollen.setPower(power);
        nectar.setPower(power);
        double inStep = t - step * holdSeconds;
        boolean settled = inStep >= holdSeconds - settleSeconds;
        double vbat = battery.getVoltage();
        double pv = pollen.getVelocity();
        double nv = nectar.getVelocity();

        sb.setLength(0);
        sb.append("t_ms=").append(System.nanoTime() / 1_000_000L)
          .append(" step=").append(step)
          .append(" power=").append(Math.round(power * 1000) / 1000.0)
          .append(" pollen_vel=").append((int) pv)
          .append(" nectar_vel=").append((int) nv)
          .append(" vbat=").append(Math.round(vbat * 100) / 100.0)
          .append(" settled=").append(settled ? 1 : 0);
        Log.d("LauncherOpenLoop", sb.toString());

        telemetry.addData("step", "%d / %d  power %.2f  %s", step + 1, steps, power, settled ? "steady" : "settling");
        telemetry.addData("pollen", "%.0f ticks/s", pv);
        telemetry.addData("nectar", "%.0f ticks/s", nv);
        telemetry.addData("battery", "%.2f V", vbat);
    }

    @Override
    public void stop() {
        pollen.setPower(0);
        nectar.setPower(0);
    }
}
