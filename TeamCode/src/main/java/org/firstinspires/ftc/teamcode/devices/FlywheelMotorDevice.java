package org.firstinspires.ftc.teamcode.devices;

import com.cdarobotics.cdalib.devices.actuators.MotorDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * A flywheel velocity loop on top of CDALib's {@link MotorDevice} (which keeps the write coalescing).
 *
 * <pre>
 *   power = (kS + kV * target + kP * error + kI * integral) * (nominalVoltage / batteryVoltage), clamped to [0, 1]
 * </pre>
 *
 * Design notes (see the Nov 2026 flywheel loop review):
 * <ul>
 *   <li>No derivative term. Hub velocity arrives in 20 ticks/s steps, so kD only amplifies quantization noise.</li>
 *   <li>Output is clamped to [0, 1]: a flywheel is never reverse-driven, it coasts down.</li>
 *   <li>The integral is clamped to {@code iMax} power and is not accumulated while the output is saturated
 *       in the direction of the error (conditional anti-windup). It is cleared whenever the target is 0.</li>
 *   <li>dt is measured from the previous update; the first update after construction uses dt = 0, so the
 *       integral never sees "seconds since boot". Gaps over 0.5 s are also skipped.</li>
 *   <li>Battery voltage is read on a timer (default every 250 ms), not every loop: the ADC read is a separate
 *       Lynx transaction, and the pack voltage does not change in 10 ms. {@code nominalVoltage <= 0} disables it.</li>
 *   <li>Gains are plain setters and may be changed on every loop, so Panels edits take effect live.</li>
 *   <li>Zero-power behavior is never queried. It is set once in init; "target 0 means power 0" needs no bus read.</li>
 * </ul>
 */
public class FlywheelMotorDevice extends MotorDevice {

    private double kS = 0.0;
    private double kV = 0.0;
    private double kP = 0.0;
    private double kI = 0.0;
    private double iMax = 0.0;            // max |kI * integral|, in power units

    private double nominalVoltage = 12.0; // <= 0 disables battery compensation
    private long voltageReadPeriodNs = 250_000_000L;
    private final VoltageSensor voltageSensor;   // null if the hardware map has none
    private double batteryVoltage = Double.NaN;
    private double voltageScale = 1.0;
    private long lastVoltageReadNs = 0;

    private double targetVelocity = 0.0;
    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTimeNs = 0;          // 0 => no previous sample yet

    // Last loop's terms, for telemetry and logging.
    private double lastVelocity = 0.0;
    private double lastFeedforward = 0.0;
    private double lastProportional = 0.0;
    private double lastIntegralTerm = 0.0;
    private double lastOutput = 0.0;

    public FlywheelMotorDevice(HardwareMap hardwareMap, String name) {
        super(hardwareMap, name);
        VoltageSensor found = null;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            found = sensor;
            break;
        }
        voltageSensor = found;
    }

    /** Sets all gains. Safe to call every loop. kV is power per tick/s at nominal voltage; iMax caps the integral's power. */
    public FlywheelMotorDevice setGains(double kS, double kV, double kP, double kI, double iMax) {
        this.kS = kS;
        this.kV = kV;
        this.kP = kP;
        this.kI = kI;
        this.iMax = Math.max(0.0, iMax);
        return this;
    }

    /** Voltage the gains were tuned at. 12 V is the usual choice; 0 or less turns compensation off. */
    public FlywheelMotorDevice setNominalVoltage(double volts) {
        nominalVoltage = volts;
        if (volts <= 0) voltageScale = 1.0;
        return this;
    }

    /** How often the battery voltage is sampled. */
    public FlywheelMotorDevice setVoltageReadPeriodMs(double millis) {
        voltageReadPeriodNs = (long) (Math.max(1.0, millis) * 1_000_000.0);
        return this;
    }

    /** Target velocity in ticks/s. 0 (or less) stops the loop: power 0, integral cleared. */
    public void setVelocity(double ticksPerSecond) {
        targetVelocity = ticksPerSecond;
    }

    public double getTargetVelocity() { return targetVelocity; }
    public double getLastVelocity() { return lastVelocity; }
    public double getLastOutput() { return lastOutput; }
    public double getLastFeedforward() { return lastFeedforward; }
    public double getLastProportional() { return lastProportional; }
    public double getLastIntegralTerm() { return lastIntegralTerm; }
    public double getBatteryVoltage() { return batteryVoltage; }
    public double getVoltageScale() { return voltageScale; }

    private void refreshVoltage(long nowNs) {
        if (voltageSensor == null || nominalVoltage <= 0) return;
        if (lastVoltageReadNs != 0 && nowNs - lastVoltageReadNs < voltageReadPeriodNs) return;
        lastVoltageReadNs = nowNs;
        double volts = voltageSensor.getVoltage();
        if (volts > 6.0) {                 // ignore a bogus 0 V read; keep the previous scale
            batteryVoltage = volts;
            voltageScale = nominalVoltage / volts;
        }
    }

    @Override
    public void update() {
        long now = System.nanoTime();
        double dt = (lastTimeNs == 0) ? 0.0 : (now - lastTimeNs) * 1e-9;
        lastTimeNs = now;
        refreshVoltage(now);

        if (targetVelocity <= 0.0) {
            integral = 0.0;
            lastError = 0.0;
            lastFeedforward = lastProportional = lastIntegralTerm = lastOutput = 0.0;
            setPower(0.0);
            super.update();
            return;
        }

        lastVelocity = getCurrentVelocity();           // bulk-cached read, one per loop
        double error = targetVelocity - lastVelocity;

        lastFeedforward = kS + kV * targetVelocity;
        lastProportional = kP * error;

        // Would integrating just push harder against a clamp we are already on? Then don't.
        double provisional = (lastFeedforward + lastProportional + kI * integral) * voltageScale;
        boolean pinnedHigh = provisional >= 1.0 && error > 0;
        boolean pinnedLow = provisional <= 0.0 && error < 0;
        if (kI > 0 && dt > 0 && dt < 0.5 && !pinnedHigh && !pinnedLow) {
            integral += 0.5 * (error + lastError) * dt;
            double limit = iMax / kI;
            if (integral > limit) integral = limit;
            else if (integral < -limit) integral = -limit;
        }
        lastError = error;
        lastIntegralTerm = kI * integral;

        double out = (lastFeedforward + lastProportional + lastIntegralTerm) * voltageScale;
        if (out > 1.0) out = 1.0;
        else if (out < 0.0) out = 0.0;
        lastOutput = out;

        setPower(out);
        super.update();
    }
}
