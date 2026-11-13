package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Where does the loop time go? A static stopwatch with named sections.
 *
 * <p>Call {@link #begin()} at the top of the OpMode's loop, {@link #mark(String)} at the end of each piece of work
 * (each subsystem's {@code update()} calls it with its own name), and {@link #end()} at the bottom. Each mark is
 * charged the time since the previous mark. One {@code Log.d("LoopProfile", ...)} line per loop reports every
 * section in ms, the total between begin and end, and the period since the previous begin; the difference between
 * period and total is what the SDK spends between loops (telemetry transmit, gamepad, Panels).
 *
 * <p>The first section after {@code begin} also carries the bulk read, because {@code ModularOpMode.loop()} clears the
 * bulk cache first and the first hardware read of the loop refills it. Disabled it costs two {@code nanoTime} calls
 * per mark. Section names should be string literals; they are compared by identity first, then equals.
 */
@Configurable
public class LoopProfiler {
    public static boolean enabled = true;
    public static int logEveryNLoops = 1;

    private static final int MAX = 16;
    private static final String[] names = new String[MAX];
    private static final long[] ns = new long[MAX];
    private static int count = 0;
    private static long loopStartNs = 0;
    private static long lastMarkNs = 0;
    private static long prevLoopStartNs = 0;
    private static long loops = 0;
    private static final StringBuilder sb = new StringBuilder(200);

    private LoopProfiler() {}

    /** Start of a loop. Resets the sections. */
    public static void begin() {
        if (!enabled) return;
        long now = System.nanoTime();
        prevLoopStartNs = loopStartNs;
        loopStartNs = now;
        lastMarkNs = now;
        for (int i = 0; i < count; i++) ns[i] = 0;
    }

    /** Charge the time since the previous mark (or begin) to {@code name}. */
    public static void mark(String name) {
        if (!enabled || loopStartNs == 0) return;
        long now = System.nanoTime();
        int idx = -1;
        for (int i = 0; i < count; i++) {
            if (names[i] == name || names[i].equals(name)) { idx = i; break; }
        }
        if (idx < 0) {
            if (count >= MAX) return;
            idx = count++;
            names[idx] = name;
        }
        ns[idx] += now - lastMarkNs;
        lastMarkNs = now;
    }

    /** End of a loop: logs the sections. */
    public static void end() {
        if (!enabled || loopStartNs == 0) return;
        long now = System.nanoTime();
        loops++;
        if (logEveryNLoops <= 0 || loops % logEveryNLoops != 0) return;
        sb.setLength(0);
        sb.append("t_ms=").append(now / 1_000_000L);
        for (int i = 0; i < count; i++) {
            sb.append(' ').append(names[i]).append('=').append(ns[i] / 100_000L / 10.0);
        }
        sb.append(" total=").append((now - loopStartNs) / 100_000L / 10.0);
        if (prevLoopStartNs != 0) sb.append(" period=").append((loopStartNs - prevLoopStartNs) / 100_000L / 10.0);
        Log.d("LoopProfile", sb.toString());
    }
}
