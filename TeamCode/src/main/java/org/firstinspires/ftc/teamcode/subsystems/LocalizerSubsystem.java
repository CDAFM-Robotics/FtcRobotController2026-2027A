package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.devices.PinpointDevice;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.util.LoopProfiler;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.PoseStorage.Alliance;

import java.util.function.BooleanSupplier;

/**
 * Pinpoint pose in Pedro field coordinates (inches; heading radians, 0 = +x, counterclockwise positive), plus the
 * TeleOp side of the autonomous handoff when set up with the 5-argument {@link #setBindings}:
 *
 *  - INIT: reads {@link PoseStorage} once. If autonomous (or an earlier TeleOp) saved a pose, it is written into the
 *    Pinpoint and its alliance is used. Otherwise the pose is "not set" and the Pinpoint is left alone.
 *  - While waiting for Start: the Pinpoint is read every loop (live pose on screen); the alliance buttons can set or
 *    change the alliance; both resets work.
 *  - Running: once the pose is known, it is saved into PoseStorage every loop.
 *  - Corner reset (hold): robot in its alliance's Garden corner, facing the left/right wall. Red (9, 9, 180 deg),
 *    Blue = 180 deg rotation (132.5, 132.5, 0 deg).
 *  - Heading reset (hold): robot facing straight away from its drivers. Red 0 deg, Blue 180 deg; x and y unchanged.
 *  - {@link #getDriveHeading()}: heading for field-centric driving. Red drivers stand at x = 0 facing +x, so Red uses
 *    the heading as is and Blue subtracts 180 deg. Returns 0 (robot-centric) until the alliance and heading are known.
 */
@Configurable
public class LocalizerSubsystem extends Subsystem {

    // Red Garden corner pose: front bumper on the left wall, left side on the bottom wall, tracking center 9 in from each.
    public static double cornerX = 9.0;
    public static double cornerY = 9.0;
    public static double fieldSize = 141.5;      // same field width Paths.java uses (mirrorX(70.75))
    public static double holdSeconds = 0.5;
    public static int resetRumbleMs = 300;
    public static int notSetRumbleMs = 600;

    private final PinpointDevice pinpoint;

    private BindingManager bindingManager;
    private boolean handoffEnabled = false;
    private Gamepad rumbleGamepad;

    private Alliance alliance = null;
    private boolean positionKnown = false;
    private boolean headingKnown = false;
    private String poseSource = "NONE";
    private boolean skipSaveOnce = false;        // the loop of a reset still holds the pre-reset reading

    private final ElapsedTime cornerHold = new ElapsedTime();
    private final ElapsedTime headingHold = new ElapsedTime();
    private boolean cornerWasHeld = false, cornerFired = false;
    private boolean headingWasHeld = false, headingFired = false;
    private String lastMessage = "";

    // Pod offsets, pod type and encoder directions all come from Pedro's Constants.localizerConfig, so TeleOp and
    // autonomous configure the Pinpoint identically. Re-run the Pinpoint Tuner and paste its block into Constants.java
    // to change them. (Nov 16: this used its own 13.25 / -9.75 cm offsets and never set the directions, so TeleOp
    // depended on whatever the previous OpMode had left on the chip.)
    public LocalizerSubsystem(HardwareMap hardwareMap, String pinpointName) {
        pinpoint = new PinpointDevice(hardwareMap, pinpointName, Constants.localizerConfig);
    }

    /** Pose only: no handoff, no resets. The "heading" analog is the raw Pinpoint heading. */
    public void setBindings(BindingManager bindingManager) {
        bindingManager.addAnalog("heading", pinpoint::getHeading);
        this.bindingManager = bindingManager;
    }

    /**
     * Pose plus the autonomous handoff, resets and alliance choice. The "heading" analog is {@link #getDriveHeading()}.
     * The alliance buttons only work before Start (gp1 B is the intake button in Competition TeleOp).
     */
    public void setBindings(BindingManager bindingManager, BooleanSupplier cornerReset, BooleanSupplier headingReset,
                            BooleanSupplier selectRed, BooleanSupplier selectBlue, Gamepad rumbleGamepad) {
        bindingManager.addAnalog("heading", this::getDriveHeading);
        bindingManager.addBinding("cornerReset", cornerReset);
        bindingManager.addBinding("headingReset", headingReset);
        bindingManager.addBinding("selectRed", selectRed);
        bindingManager.addBinding("selectBlue", selectBlue);
        this.bindingManager = bindingManager;
        this.rumbleGamepad = rumbleGamepad;
        handoffEnabled = true;
    }

    /** X position in inches */
    public double getX() { return pinpoint.getX(); }

    /** Y position in inches */
    public double getY() { return pinpoint.getY(); }

    /** Heading in radians */
    public double getHeading() { return pinpoint.getHeading(); }

    /** Null until set by PoseStorage or the alliance buttons. */
    public Alliance getAlliance() { return alliance; }

    public boolean isFieldCentric() { return alliance != null && headingKnown; }

    /** Position, heading and alliance all known: what auto-aim needs. */
    public boolean isPoseKnown() { return alliance != null && positionKnown && headingKnown; }

    /** Heading for field-centric driving (radians); 0 = robot-centric until the alliance and heading are known. */
    public double getDriveHeading() {
        if (!isFieldCentric()) return 0;
        return alliance == Alliance.RED ? pinpoint.getHeading() : pinpoint.getHeading() - Math.PI;
    }

    @Override
    public void init() {
        if (handoffEnabled && PoseStorage.hasPose()) {
            pinpoint.setPose(PoseStorage.getX(), PoseStorage.getY(), PoseStorage.getHeading());
            alliance = PoseStorage.getAlliance();
            positionKnown = true;
            headingKnown = true;
            poseSource = String.format("%s (%.0f s old at INIT)",
                    PoseStorage.getSource() == PoseStorage.Source.AUTONOMOUS ? "autonomous" : "last TeleOp",
                    PoseStorage.ageSeconds());
            skipSaveOnce = true;
        }
        pinpoint.update();
        if (handoffEnabled && alliance == null && rumbleGamepad != null) {
            rumbleGamepad.rumble(notSetRumbleMs);
        }
    }

    @Override
    public void init_loop() {
        pinpoint.update();
        if (!handoffEnabled) return;

        if (bindingManager.checkBinding("selectRed")) alliance = Alliance.RED;
        if (bindingManager.checkBinding("selectBlue")) alliance = Alliance.BLUE;
        handleResets();
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        pinpoint.update();
        if (handoffEnabled) {
            handleResets();
            if (skipSaveOnce) {
                skipSaveOnce = false;
            } else if (positionKnown && headingKnown && alliance != null) {
                PoseStorage.save(pinpoint.getX(), pinpoint.getY(), pinpoint.getHeading(), alliance, PoseStorage.Source.TELEOP);
            }
        }
        LoopProfiler.mark("localizer");
    }

    /** Fires each reset once per press, after the button has been held for holdSeconds. */
    private void handleResets() {
        boolean cornerHeld = bindingManager.checkBinding("cornerReset");
        if (cornerHeld && !cornerWasHeld) { cornerHold.reset(); cornerFired = false; }
        if (cornerHeld && !cornerFired && cornerHold.seconds() >= holdSeconds) {
            cornerFired = true;
            if (alliance == null) {
                lastMessage = "Corner reset ignored: set the alliance first (B / X during INIT)";
            } else if (alliance == Alliance.RED) {
                applyReset(cornerX, cornerY, Math.PI, "corner reset");
            } else {
                applyReset(fieldSize - cornerX, fieldSize - cornerY, 0, "corner reset");
            }
        }
        cornerWasHeld = cornerHeld;

        boolean headingHeld = bindingManager.checkBinding("headingReset");
        if (headingHeld && !headingWasHeld) { headingHold.reset(); headingFired = false; }
        if (headingHeld && !headingFired && headingHold.seconds() >= holdSeconds) {
            headingFired = true;
            if (alliance == null) {
                lastMessage = "Heading reset ignored: set the alliance first (B / X during INIT)";
            } else {
                double h = alliance == Alliance.RED ? 0 : Math.PI;
                pinpoint.setPose(pinpoint.getX(), pinpoint.getY(), h);
                headingKnown = true;
                if (!positionKnown) poseSource = "heading reset only (x, y not set)";
                skipSaveOnce = true;
                lastMessage = "Heading reset";
                if (rumbleGamepad != null) rumbleGamepad.rumble(resetRumbleMs);
            }
        }
        headingWasHeld = headingHeld;
    }

    private void applyReset(double x, double y, double heading, String source) {
        pinpoint.setPose(x, y, heading);
        positionKnown = true;
        headingKnown = true;
        poseSource = source;
        skipSaveOnce = true;
        lastMessage = "Corner reset";
        if (rumbleGamepad != null) rumbleGamepad.rumble(resetRumbleMs);
    }

    /** Pose and handoff lines for the Driver Station; call from the OpMode's init_loop and loop. */
    public void addTelemetry(Telemetry telemetry) {
        if (alliance == null) {
            telemetry.addLine("ALLIANCE: NOT SET - gp1 B = Red, X = Blue (before Start)");
        } else {
            telemetry.addLine("ALLIANCE: " + alliance);
        }
        telemetry.addData("Pose x / y / heading", "%.1f in / %.1f in / %.1f deg",
                pinpoint.getX(), pinpoint.getY(), Math.toDegrees(pinpoint.getHeading()));
        telemetry.addData("Pose from", positionKnown || headingKnown ? poseSource : "NONE - do a corner reset");
        telemetry.addData("Drive", isFieldCentric() ? "FIELD-CENTRIC" : "robot-centric (alliance or heading not set)");
        if (!lastMessage.isEmpty()) telemetry.addLine(lastMessage);
        telemetry.addLine("gp1 hold Back/Share: corner reset | hold dpad down: heading reset");
    }

    @Override
    public void stop() {

    }
}
