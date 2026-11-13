package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.util.LoopProfiler;

/**
 * Closed-loop step test of the launcher, no gamepad needed: both wheels on at their targets for onSeconds, off for
 * offSeconds, repeated cycles times, then the OpMode stops itself. Spin-up time, overshoot and hold come straight
 * out of the LauncherVelocity log, which carries target, actual, power and battery voltage every loop.
 */
@Configurable
@TeleOp(name = "Launcher Step Test", group = "Testing")
public class LauncherStepTest extends ModularOpMode {

    public static double onSeconds = 6.0;
    public static double offSeconds = 4.0;
    public static int cycles = 2;

    private LauncherSubsystem launcherSubsystem;
    private long startNs = 0;
    private boolean on = false;
    private boolean done = false;

    @Override
    protected void preload() {
        launcherSubsystem = new LauncherSubsystem(hardwareMap, telemetry, "pollenLauncherMotor", "nectarLauncherMotor");
        launcherSubsystem.setBindings(bindingManager, () -> on, () -> on);
        registerSubsystem(launcherSubsystem);
    }

    @Override
    public void start() {
        super.start();
        startNs = System.nanoTime();
    }

    @Override
    public void loop() {
        LoopProfiler.begin();
        double t = (System.nanoTime() - startNs) * 1e-9;
        double period = onSeconds + offSeconds;
        int cycle = (int) (t / period);
        if (cycle >= cycles) {
            on = false;
            if (!done) { done = true; requestOpModeStop(); }
        } else {
            on = (t - cycle * period) < onSeconds;
        }
        telemetry.addData("phase", "%s  cycle %d/%d  t=%.1f", on ? "ON" : "off", Math.min(cycle + 1, cycles), cycles, t);
        LoopProfiler.mark("opmode");
        super.loop();
        LoopProfiler.end();
        telemetry.update();
    }
}
