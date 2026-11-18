package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

/**
 * Hands the robot's pose (Pedro field coordinates: inches, heading in radians) and alliance from autonomous to TeleOp.
 *
 * Static fields live as long as the Robot Controller app, so what autonomous saves is still here when TeleOp is
 * INITed. It is lost on a power cycle, a hub reboot, an app restart or a new deploy; TeleOp then reports "no pose".
 * Autonomous saves every loop; TeleOp reads once at INIT and then saves every loop, so a TeleOp restart continues
 * from its own latest pose.
 */
public final class PoseStorage {

    public enum Alliance { RED, BLUE }

    /** Who wrote the pose last. */
    public enum Source { AUTONOMOUS, TELEOP }

    private static boolean hasPose = false;
    private static double x, y, heading;
    private static Alliance alliance = null;
    private static Source source = null;
    private static long savedAtNs = 0;

    private PoseStorage() { }

    public static void save(double xIn, double yIn, double headingRad, Alliance a, Source s) {
        x = xIn;
        y = yIn;
        heading = headingRad;
        alliance = a;
        source = s;
        savedAtNs = System.nanoTime();
        hasPose = true;
    }

    /** One line for an autonomous loop: {@code PoseStorage.saveFromAutonomous(follower.pose(), PoseStorage.Alliance.RED);} */
    public static void saveFromAutonomous(Pose pose, Alliance a) {
        save(pose.x(), pose.y(), pose.heading(), a, Source.AUTONOMOUS);
    }

    public static boolean hasPose() { return hasPose; }
    public static double getX() { return x; }
    public static double getY() { return y; }
    public static double getHeading() { return heading; }
    /** May be null if nothing has been saved. */
    public static Alliance getAlliance() { return alliance; }
    /** May be null if nothing has been saved. */
    public static Source getSource() { return source; }

    /** Seconds since the last save; only meaningful when {@link #hasPose()}. */
    public static double ageSeconds() {
        return (System.nanoTime() - savedAtNs) / 1e9;
    }
}
