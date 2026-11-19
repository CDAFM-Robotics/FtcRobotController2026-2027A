package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.devices.actuators.CRServoDevice;
import com.cdarobotics.cdalib.devices.actuators.MotorDevice;
import com.cdarobotics.cdalib.devices.sensors.RevColorSensorV3Device;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.util.LoopProfiler;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Configurable
public class TransferSubsystem extends Subsystem {

    // The SDK leaves I2C devices at 100 kHz unless the driver asks for more; the Pinpoint driver asks for 400 kHz,
    // the REV Color Sensor V3 driver does not, although the APDS-9151 supports it. Applied at construction.
    public static boolean colorSensorFast400k = true;

    private static final double TRANSFER_POWER_STEP = 0.1;
    // Feed power. All three are Panels fields and are read every loop, so an edit in Panels takes effect at once.
    // The 5-argument setBindings can also step transferMotorPower by 0.1 from buttons; no OpMode uses it now
    // (WholeBotTest dropped its gp2 dpad left/right stepping on Nov 13: the servos were seen running while the motor was not, and a lowered transferMotorPower is the suspected cause).
    // Nov 6-7 experiment: balls that touch in the transfer strike the flywheel 40-140 ms apart, faster than the wheel
    // recovers, so the plan is to slow the feed to space them out. Tested so far (valid runs only): motor 0.4 with
    // servos 0.4, and motor 0.4 with servos 1.0, both moved one ball per trigger. Motor powers above 0.4 were never
    // actually applied (a Panels copy bug, fixed with this field), and the servos were never tested below 1.0 with
    // the motor at 1.0. Defaults are full power until the sweep is done.
    public static double transferMotorPower = 1.0;   // transfer belt motor while the feed binding is held
    public static double transferServoPower = 1.0;   // the two CR feed servos
    public static double autoNudgePower = 1.0;       // power when a ball is detected and nothing else is running
                                                     // (0.5 stalled the belt motor against a ball, Nov 14)
    // Reverse step (one ball space): motor only, backwards, until the encoder has moved reverseStepTicks.
    public static double reverseNudgePower = 1.0;    // motor power (run backwards) for the reverse step; servos stay off
    public static int reverseStepTicks = 150;        // encoder ticks = one ball space (tune: see "last reverse step" telemetry)
    public static double reverseStepTimeoutSec = 0.5; // safety stop if the encoder doesn't reach the count (jam, unplugged)

    private final MotorDevice transferMotor;
    private final CRServoDevice leftTransferServo;
    private final CRServoDevice rightTransferServo;
    // Both sensors go through CDALib's RevColorSensorV3Device: one 14-byte I2C read per sensor per loop in update(),
    // after which distance and colour are cached. The stock NormalizedColorSensor/DistanceSensor path cost one I2C
    // transaction per getDistance()/getNormalizedColors() call, and this subsystem was issuing up to eleven per loop.
    private final RevColorSensorV3Device colorSensor1;
    private final RevColorSensorV3Device colorSensor2;
    // topTransferSensor: REV Color Sensor V3 at the top of the motor zone (Expansion Hub 2 I2C bus 1). Optional.
    private final RevColorSensorV3Device topSensor;
    // Ball at the top when closer than this. Measured Nov 14 with TopTransferSensorTest: Pollen parked at the top
    // 1.83-1.86 cm; empty and a ball in the second motor-zone slot 4.99-5.29 cm. Midpoint about 3.42 cm.
    public static double topBallDistanceCm = 3.42;
    private boolean sensorsRead = false;   // cached values are meaningless before the first update()
    private static final double GAME_ELEMENT_DISTANCE_CM  = 5.0;
    private static final double GAME_ELEMENT_DISTANCE2_CM = 6.0;

    private BindingManager bindingManager;
    // When true, the ball-detect auto-nudge is suppressed (the explicit transferMotor/transferServo bindings still
    // run). The shooting OpMode sets this to "launcher enabled and wheel not yet back at speed" so stacked balls wait
    // for the flywheel instead of being nudged into it at 0.5 power.
    private BooleanSupplier feedHold = () -> false;
    private boolean lastHold = false;

    public void setFeedHold(BooleanSupplier feedHold) {
        this.feedHold = feedHold;
    }

    // Extra nudge request from the OpMode (e.g. a manual nudge toggle). Treated like a detected ball: runs the motor at
    // autoNudgePower when the transferMotor binding is off, and is suppressed by feedHold like the auto-nudge.
    private BooleanSupplier nudgeRequest = () -> false;

    public void setNudgeRequest(BooleanSupplier nudgeRequest) {
        this.nudgeRequest = nudgeRequest;
    }

    // Reverse step: ONLY the motor runs backwards at reverseNudgePower, servos off, until the encoder has moved
    // reverseStepTicks (or reverseStepTimeoutSec passes). Overrides the auto-nudge; a launch feed cancels it.
    private boolean reversing = false;
    private int reverseStartPos = 0;
    private int lastReverseTicks = 0;
    private boolean lastReverseTimedOut = false;
    private final com.qualcomm.robotcore.util.ElapsedTime reverseTimer = new com.qualcomm.robotcore.util.ElapsedTime();

    /** Start one reverse step (one ball space). Ignored if one is already running. */
    public void startReverseStep() {
        if (reversing) return;
        reversing = true;
        reverseStartPos = transferMotor.getCurrentPosition();
        reverseTimer.reset();
    }

    public boolean isReversing() { return reversing; }
    /** Ticks the last finished reverse step actually moved, and whether it stopped on the timeout. */
    public int getLastReverseTicks() { return lastReverseTicks; }
    public boolean didLastReverseTimeOut() { return lastReverseTimedOut; }
    public int getMotorPosition() { return transferMotor.getCurrentPosition(); }

    /** @return true if the auto-nudge was being held back on the last update(). */
    public boolean isFeedHeld() { return lastHold; }

    public TransferSubsystem(HardwareMap hardwareMap, String transferMotorName, String leftTransferServoName, String rightTransferServoName){
        transferMotor = new MotorDevice(hardwareMap, transferMotorName);
        leftTransferServo = new CRServoDevice(hardwareMap, leftTransferServoName);
        rightTransferServo = new CRServoDevice(hardwareMap, rightTransferServoName);
        RevColorSensorV3 cs1 = hardwareMap.tryGet(RevColorSensorV3.class, "colorSensor1");
        colorSensor1 = (cs1 == null) ? null : new RevColorSensorV3Device(cs1).setGain(10);

        RevColorSensorV3 cs2 = hardwareMap.tryGet(RevColorSensorV3.class, "colorSensor2");
        colorSensor2 = (cs2 == null) ? null : new RevColorSensorV3Device(cs2);

        RevColorSensorV3 top = hardwareMap.tryGet(RevColorSensorV3.class, "topTransferSensor");
        topSensor = (top == null) ? null : new RevColorSensorV3Device(top);

        if (colorSensorFast400k) {
            setBusSpeed400k(cs1);
            setBusSpeed400k(cs2);
            setBusSpeed400k(top);
        }
    }

    private static void setBusSpeed400k(RevColorSensorV3 sensor) {
        if (sensor == null) return;
        I2cDeviceSynchSimple client = sensor.getDeviceClient();
        if (client instanceof LynxI2cDeviceSynch) {
            ((LynxI2cDeviceSynch) client).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
        }
    }

    /** Cached from this loop's update(); free to call as often as you like. */
    public boolean hasGameElement() {
        if (!sensorsRead) return false;
        boolean sensor1 = colorSensor1 != null && colorSensor1.getDistanceCm() < GAME_ELEMENT_DISTANCE_CM;
        boolean sensor2 = colorSensor2 != null && colorSensor2.getDistanceCm() < GAME_ELEMENT_DISTANCE2_CM;
        return sensor1 || sensor2;
    }

    /** Cached from this loop's update(): colorSensor1 alone sees a ball (closer than GAME_ELEMENT_DISTANCE_CM). */
    public boolean hasBallAtSensor1() {
        return sensorsRead && colorSensor1 != null && colorSensor1.getDistanceCm() < GAME_ELEMENT_DISTANCE_CM;
    }

    /**
     * Reads topTransferSensor now (one I2C read per call; it is not read in update()) and returns true if a ball is
     * at the top of the transfer (closer than topBallDistanceCm). False if the sensor is not configured.
     */
    public boolean hasBallAtTop() {
        if (topSensor == null) return false;
        topSensor.update();
        return topSensor.getDistanceCm() < topBallDistanceCm;
    }

    /** Cached distance from colorSensor1 in cm, or NaN if absent or not yet read. */
    public double getDistance() {
        if (colorSensor1 == null || !sensorsRead) return Double.NaN;
        return colorSensor1.getDistanceCm();
    }

    /** Cached distance from colorSensor2 in cm, or NaN if absent or not yet read. */
    public double getDistance2() {
        if (colorSensor2 == null || !sensorsRead) return Double.NaN;
        return colorSensor2.getDistanceCm();
    }

    /** Cached normalized colour from colorSensor1, or null if absent. */
    public NormalizedRGBA getColorReading() {
        if (colorSensor1 == null) return null;
        return colorSensor1.getColors();
    }

    public void setBindings(BindingManager bindingManager, BooleanSupplier transferServo, BooleanSupplier transferMotor) {
        bindingManager.addBinding("transferServo", transferServo);
        bindingManager.addBinding("transferMotor", transferMotor);
        bindingManager.addBinding("increaseTransferPower", () -> false);
        bindingManager.addBinding("decreaseTransferPower", () -> false);
        this.bindingManager = bindingManager;
    }

    public void setBindings(BindingManager bindingManager, BooleanSupplier transferServo, BooleanSupplier transferMotor, BooleanSupplier increaseTransferPower, BooleanSupplier decreaseTransferPower) {
        bindingManager.addBinding("transferServo", transferServo);
        bindingManager.addBinding("transferMotor", transferMotor);
        bindingManager.addBinding("increaseTransferPower", increaseTransferPower);
        bindingManager.addBinding("decreaseTransferPower", decreaseTransferPower);
        this.bindingManager = bindingManager;
    }

    public double getTransferMotorPower() { return transferMotorPower; }

    // What update() actually commanded this loop, for telemetry and the TransferState log.
    public enum Mode { OFF, FULL, NUDGE, HELD, REVERSE }
    private Mode lastMode = Mode.OFF;
    private double lastMotorCmd = 0, lastServoCmd = 0;
    private Mode loggedMode = null;
    private double loggedMotor = Double.NaN, loggedServo = Double.NaN;

    /** FULL: transferMotor binding on (full feed power). NUDGE: ball detected / nudge request. HELD: feedHold is
     *  suppressing a nudge. REVERSE: motor backwards for the reverse nudge. OFF: nothing. */
    public Mode getMode() { return lastMode; }
    public double getMotorCommand() { return lastMotorCmd; }
    public double getServoCommand() { return lastServoCmd; }

    @Override
    public void init() {
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // BRAKE so a queued ball stops when the feed gate closes instead of coasting on into the flywheel.
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTransferServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    private boolean prevIncreaseTransferPower = false;
    private boolean prevDecreaseTransferPower = false;

    @Override
    public void update() {
        // One I2C read per sensor per loop; every getter below (and the OpMode's binding lambdas and telemetry)
        // reads the cached result.
        if (colorSensor1 != null) colorSensor1.update();
        if (colorSensor2 != null) colorSensor2.update();
        sensorsRead = true;

        boolean increaseNow = bindingManager.checkBinding("increaseTransferPower");
        boolean decreaseNow = bindingManager.checkBinding("decreaseTransferPower");
        if (increaseNow && !prevIncreaseTransferPower)
            transferMotorPower = Math.min(1.0, transferMotorPower + TRANSFER_POWER_STEP);
        if (decreaseNow && !prevDecreaseTransferPower)
            transferMotorPower = Math.max(0.0, transferMotorPower - TRANSFER_POWER_STEP);
        prevIncreaseTransferPower = increaseNow;
        prevDecreaseTransferPower = decreaseNow;

        boolean manualRun = bindingManager.checkBinding("transferMotor");
        boolean hold = feedHold.getAsBoolean();
        lastHold = hold;
        boolean elementDetected = (hasGameElement() || nudgeRequest.getAsBoolean()) && !hold;   // nudge only when not held

        if (reversing) {
            int moved = Math.abs(transferMotor.getCurrentPosition() - reverseStartPos);
            boolean timedOut = reverseTimer.seconds() >= reverseStepTimeoutSec;
            if (moved >= reverseStepTicks || timedOut || manualRun) {
                reversing = false;
                lastReverseTicks = moved;
                lastReverseTimedOut = timedOut && moved < reverseStepTicks;
            }
        }
        boolean reverse = reversing;

        double motorCmd = manualRun ? transferMotorPower : (reverse ? -reverseNudgePower : (elementDetected ? autoNudgePower : 0));
        boolean servoRun = bindingManager.checkBinding("transferServo");
        double servoCmd = reverse ? 0 : (servoRun ? transferServoPower : (elementDetected ? autoNudgePower : 0));

        transferMotor.setPower(motorCmd);
        leftTransferServo.setPower(servoCmd);
        rightTransferServo.setPower(servoCmd);

        boolean ballSeen = hasGameElement() || nudgeRequest.getAsBoolean();
        lastMode = manualRun ? Mode.FULL : (reverse ? Mode.REVERSE
                : (elementDetected ? Mode.NUDGE : ((hold && ballSeen) ? Mode.HELD : Mode.OFF)));
        lastMotorCmd = motorCmd;
        lastServoCmd = servoCmd;
        if (lastMode != loggedMode || motorCmd != loggedMotor || servoCmd != loggedServo) {
            Log.d("TransferState", String.format("t_ms=%d mode=%s motor=%.2f servo=%.2f ball=%d hold=%d",
                    System.nanoTime() / 1_000_000L, lastMode, motorCmd, servoCmd, hasGameElement() ? 1 : 0, hold ? 1 : 0));
            loggedMode = lastMode; loggedMotor = motorCmd; loggedServo = servoCmd;
        }

        transferMotor.update();

        leftTransferServo.update();
        rightTransferServo.update();
        LoopProfiler.mark("transfer");
    }

    @Override
    public void stop() {

    }
}
