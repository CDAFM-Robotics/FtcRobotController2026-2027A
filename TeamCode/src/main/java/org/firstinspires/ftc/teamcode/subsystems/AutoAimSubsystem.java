package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.subsystems.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PoseStorage.Alliance;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * TeleOp auto-aim at our alliance's up-facing HIVE CELL, and launcher velocity from distance.
 *
 * Field (Pedro inches; Red wall x = 0, audience wall y = 0). From the BIOBUZZ manual (Section 9, TU06): the HIVE
 * Structure is at the field center, the red and blue HIVES are 25.5 in apart center to center (red toward the red
 * wall), and the up CELL's opening faces outward (toward the audience wall or the far wall), centered ~59.6 in above
 * the tiles. The opening's x/y below are worked out from the manual's figures (~+/-2 in).
 *
 * We always target the CELL on the robot's half of the field (audience or far side, from the pose); nothing reads the
 * camera and no button selects it.
 *
 * Robot: both launchers fire out the robot's RIGHT side (shot direction = heading - 90 deg). Along the robot's length
 * (from CAD, 18 in robot, tracking center 9 in from the front): Pollen launcher 6.148 in behind the center, Nectar
 * 0.623 in ahead. Aim puts the line halfway between the two launchers (aimBlend = 0.5) through the CELL, so each
 * launcher's parallel shot line is ~3.4 in to either side of it.
 *
 * Velocity: each launcher's base target comes from its own table of (distance from that launcher to the CELL ->
 * ticks/s), linear between points and flat past the ends (measured Nov 18). Nectar also gets a shot-angle correction,
 * since straight-on shots need less speed than angled ones. Without a known pose the base velocities
 * are NaN and LauncherSubsystem falls back to its own fixed defaults.
 */
@Configurable
public class AutoAimSubsystem extends Subsystem {

    // CELL opening centers (x of each HIVE, y of each CELL side)
    public static double redHiveX = 58.0;
    public static double blueHiveX = 83.5;
    public static double audienceCellY = 55.0;
    public static double farCellY = 86.5;

    // Launcher positions along the robot's forward axis, inches from the tracking center (+ = forward)
    public static double pollenForwardOffset = -6.148;
    public static double nectarForwardOffset = 0.623;
    // Which point along the robot the aim line goes through: 0 = Pollen, 1 = Nectar, 0.5 = halfway between them
    public static double aimBlend = 0.5;

    // Heading controller: turn = kP * error + kS (when outside the deadband), capped at maxTurn
    public static double turnKP = 1.2;           // turn power per radian of error
    public static double turnKS = 0.10;          // minimum turn power to overcome friction
    public static double turnMax = 0.7;
    public static double deadbandDeg = 0.5;
    public static double aimedToleranceDeg = 2.0;

    // Distance (in) -> velocity (ticks/s) tables, 5 points each, flat past the ends. Edit live in Panels.
    // Measured Nov 18 2026 (Red audience CELL). ~60 in is about the farthest shot (field corner), ~43 in the farthest
    // straight-on shot. Pollen: same speed straight or angled. Nectar: these are ANGLED (~50 deg) shots; straight-on
    // shots need ~70 less, handled by the angle correction below.
    public static double dist1 = 33, dist2 = 44, dist3 = 50, dist4 = 55, dist5 = 60;
    public static double pollenVel1 = 1300, pollenVel2 = 1320, pollenVel3 = 1340, pollenVel4 = 1380, pollenVel5 = 1440;
    public static double nectarVel1 = 1480, nectarVel2 = 1540, nectarVel3 = 1540, nectarVel4 = 1540, nectarVel5 = 1560;

    // Nectar angle correction: + nectarAngleGain * (angle - nectarTableAngleDeg) / nectarTableAngleDeg, where angle is how
    // far the shot comes in off straight-on to the CELL opening (0 = straight in front, the opening faces the wall).
    // 0 deg -> -70, 50 deg -> 0. 0 gain turns it off.
    public static double nectarAngleGain = 70;
    public static double nectarTableAngleDeg = 50;

    public enum CellSide { AUDIENCE, FAR }

    private final PoseSource pose;
    private BindingManager bindingManager;
    private DoubleSupplier manualTurn;

    private double turnCommand = 0;
    private double headingErrorRad = Double.NaN;
    private boolean aiming = false;

    /**
     * @param pose field pose + alliance source: {@link LocalizerSubsystem} in TeleOp, {@link FollowerPoseSource} (the
     *             Pedro Follower) in autonomous.
     */
    public AutoAimSubsystem(PoseSource pose) {
        this.pose = pose;
    }

    /**
     * The CELL we aim at: whichever of our two is on the robot's half of the field. The two CELLS sit at audienceCellY
     * and farCellY, so the boundary is halfway between them - a robot nearer the audience wall targets the audience
     * CELL, otherwise the far CELL. Picked from the pose every loop; no driver input selects it.
     */
    public CellSide getTargetCell() {
        return pose.getY() < (audienceCellY + farCellY) / 2 ? CellSide.AUDIENCE : CellSide.FAR;
    }

    /**
     * @param aimHold     held: the robot turns itself so the Pollen line faces the CELL on its half of the field
     * @param manualTurn  the driver's turn stick, used whenever aim is not active
     */
    public void setBindings(BindingManager bindingManager, BooleanSupplier aimHold, DoubleSupplier manualTurn) {
        bindingManager.addBinding("aimHold", aimHold);
        this.bindingManager = bindingManager;
        this.manualTurn = manualTurn;
    }

    /** Turn command for the drivetrain: auto while aiming, otherwise the driver's stick (0 when no stick is wired). */
    public double getTurn() {
        if (aiming) return turnCommand;
        return manualTurn != null ? manualTurn.getAsDouble() : 0;
    }

    public boolean isAvailable() { return pose.isPoseKnown(); }

    /** True while the aim button is held and the robot is turning itself. */
    public boolean isAiming() { return aiming; }

    /** Heading error in degrees (NaN when unavailable or too close). */
    public double getHeadingErrorDeg() { return Math.toDegrees(headingErrorRad); }

    public boolean isAimed() {
        return isAvailable() && !Double.isNaN(headingErrorRad) && Math.abs(Math.toDegrees(headingErrorRad)) <= aimedToleranceDeg;
    }

    public double getTargetX() { return pose.getAlliance() == Alliance.BLUE ? blueHiveX : redHiveX; }
    public double getTargetY() { return getTargetCell() == CellSide.AUDIENCE ? audienceCellY : farCellY; }

    /** Distance (in) from the launcher at the given forward offset to the CELL, at the robot's current pose. */
    private double launcherDistance(double forwardOffset) {
        double h = pose.getHeading();
        double lx = pose.getX() + forwardOffset * Math.cos(h);
        double ly = pose.getY() + forwardOffset * Math.sin(h);
        return Math.hypot(getTargetX() - lx, getTargetY() - ly);
    }

    public double getPollenDistance() { return launcherDistance(pollenForwardOffset); }
    public double getNectarDistance() { return launcherDistance(nectarForwardOffset); }

    /** Base Pollen velocity from the table; NaN when the pose is not known (the launcher then uses its default). */
    public double getPollenBaseVelocity() {
        if (!isAvailable()) return Double.NaN;
        return interpolate(getPollenDistance(), pollenVel1, pollenVel2, pollenVel3, pollenVel4, pollenVel5);
    }

    public double getNectarBaseVelocity() {
        if (!isAvailable()) return Double.NaN;
        return interpolate(getNectarDistance(), nectarVel1, nectarVel2, nectarVel3, nectarVel4, nectarVel5)
                + getNectarAngleCorrection();
    }

    /**
     * Degrees the shot comes in off straight-on: the angle between the CELL's facing (audience CELL faces the audience
     * wall, -y; far CELL faces the far wall, +y) and the line from the CELL to the point between the launchers. 0..180.
     */
    public double getShotAngleDeg() {
        double h = pose.getHeading();
        double mid = pollenForwardOffset + aimBlend * (nectarForwardOffset - pollenForwardOffset);
        double vx = pose.getX() + mid * Math.cos(h) - getTargetX();
        double vy = pose.getY() + mid * Math.sin(h) - getTargetY();
        double facingY = getTargetCell() == CellSide.AUDIENCE ? -1 : 1;
        double len = Math.hypot(vx, vy);
        if (len < 1e-6) return 0;
        return Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, vy * facingY / len))));
    }

    /** Nectar speed change for the shot angle (ticks/s); 0 at nectarTableAngleDeg, negative straight on. */
    public double getNectarAngleCorrection() {
        if (nectarTableAngleDeg <= 0) return 0;
        double angle = Math.min(90, getShotAngleDeg());
        return nectarAngleGain * (angle - nectarTableAngleDeg) / nectarTableAngleDeg;
    }

    private static double interpolate(double d, double v1, double v2, double v3, double v4, double v5) {
        double[] ds = {dist1, dist2, dist3, dist4, dist5};
        double[] vs = {v1, v2, v3, v4, v5};
        if (d <= ds[0]) return vs[0];
        for (int i = 1; i < ds.length; i++) {
            if (d <= ds[i]) {
                double span = ds[i] - ds[i - 1];
                double t = span > 0 ? (d - ds[i - 1]) / span : 1;
                return vs[i - 1] + t * (vs[i] - vs[i - 1]);
            }
        }
        return vs[vs.length - 1];
    }

    /**
     * Heading (rad) that puts the aim line (between Pollen and Nectar, set by aimBlend) through the CELL. The shot leaves the launcher toward the robot's
     * right, so every point on the line has the same forward coordinate as the launcher (a). With D = CELL - center at
     * angle phi: D . forward = a  =>  heading = phi + acos(a / |D|) (the + root keeps the CELL on the right side).
     * NaN if the robot is closer to the CELL than |a|.
     */
    private double desiredHeading() {
        double dx = getTargetX() - pose.getX();
        double dy = getTargetY() - pose.getY();
        double dist = Math.hypot(dx, dy);
        double a = pollenForwardOffset + aimBlend * (nectarForwardOffset - pollenForwardOffset);
        if (dist <= Math.abs(a) + 1e-6) return Double.NaN;
        return Math.atan2(dy, dx) + Math.acos(a / dist);
    }

    private static double wrap(double rad) {
        while (rad > Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }

    @Override
    public void init() { }

    @Override
    public void init_loop() { }

    @Override
    public void start() { }

    @Override
    public void update() {
        if (isAvailable()) {
            double want = desiredHeading();
            headingErrorRad = Double.isNaN(want) ? Double.NaN : wrap(want - pose.getHeading());
        } else {
            headingErrorRad = Double.NaN;
        }

        // Autonomous registers this subsystem (so update() runs) without aim bindings; with no BindingManager it simply
        // never auto-turns - the launcher still reads the distance-based velocity getters, which don't need update().
        aiming = bindingManager != null && bindingManager.checkBinding("aimHold") && !Double.isNaN(headingErrorRad);
        if (aiming) {
            double e = headingErrorRad;
            double u = 0;
            if (Math.abs(Math.toDegrees(e)) > deadbandDeg) {
                u = turnKP * e + Math.signum(e) * turnKS;
                u = Math.max(-turnMax, Math.min(turnMax, u));
            }
            // Positive turn stick turns the robot clockwise (heading decreases), so a positive error needs negative turn.
            turnCommand = -u;
        } else {
            turnCommand = 0;
        }
    }

    private String cellName() {
        return (pose.getAlliance() == Alliance.BLUE ? "BLUE " : "RED ") + getTargetCell() + " CELL";
    }

    /** Aim lines for the Driver Station. */
    public void addTelemetry(Telemetry telemetry) {
        if (!isAvailable()) {
            telemetry.addLine("AIM: unavailable (alliance / pose not set) - fixed launcher speeds");
            return;
        }
        telemetry.addLine("TARGET: " + cellName() + "  (auto: the CELL on your half of the field)");
        String aimState;
        if (Double.isNaN(headingErrorRad)) aimState = "too close to the CELL";
        else if (isAimed()) aimState = "AIMED";
        else aimState = String.format("off by %.1f deg", Math.toDegrees(headingErrorRad));
        telemetry.addData("Aim (hold gp2 X)", "%s%s", aimState, aiming ? "  [auto-turning]" : "");
        telemetry.addData("Pollen dist / base vel", "%.1f in / %.0f", getPollenDistance(), getPollenBaseVelocity());
        telemetry.addData("Nectar dist / base vel", "%.1f in / %.0f", getNectarDistance(), getNectarBaseVelocity());
        telemetry.addData("Shot angle / Nectar adj", "%.0f deg / %+.0f", getShotAngleDeg(), getNectarAngleCorrection());
    }

    @Override
    public void stop() { }
}
