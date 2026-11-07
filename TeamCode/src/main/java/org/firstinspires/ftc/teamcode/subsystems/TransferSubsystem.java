package org.firstinspires.ftc.teamcode.subsystems;

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
    // Feed speed experiment (Nov 6): balls that touch in the transfer strike the flywheel 40-140 ms apart, faster than
    // the ~250 ms the wheel needs to recover and faster than the per-shot gate can react. Spacing in time is one ball
    // diameter over the transfer's linear speed, so the transfer has to run several times slower, around 0.3 m/s,
    // for the gate to catch the second ball. Motor power is the dpad adjustment; the servos were fixed at full power,
    // which would have defeated slowing the motor alone. All three are Panels fields now.
    public static double transferMotorPowerDefault = 1.0;   // starting value for the dpad-adjusted motor power
    public static double transferServoPower = 1.0;          // the two CR feed servos
    public static double autoNudgePower = 0.5;              // power when a ball is detected and nothing else is running

    private double transferMotorPower = transferMotorPowerDefault;

    private final MotorDevice transferMotor;
    private final CRServoDevice leftTransferServo;
    private final CRServoDevice rightTransferServo;
    // Both sensors go through CDALib's RevColorSensorV3Device: one 14-byte I2C read per sensor per loop in update(),
    // after which distance and colour are cached. The stock NormalizedColorSensor/DistanceSensor path cost one I2C
    // transaction per getDistance()/getNormalizedColors() call, and this subsystem was issuing up to eleven per loop.
    private final RevColorSensorV3Device colorSensor1;
    private final RevColorSensorV3Device colorSensor2;
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

        if (colorSensorFast400k) {
            setBusSpeed400k(cs1);
            setBusSpeed400k(cs2);
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
        boolean elementDetected = hasGameElement() && !hold;   // auto-nudge only when not held

        if (manualRun) {
            transferMotor.setPower(transferMotorPower);
        } else if (elementDetected) {
            transferMotor.setPower(autoNudgePower);
        } else {
            transferMotor.setPower(0);
        }

        if (bindingManager.checkBinding("transferServo")) {
            leftTransferServo.setPower(transferServoPower);
            rightTransferServo.setPower(transferServoPower);
        } else if (elementDetected) {
            leftTransferServo.setPower(autoNudgePower);
            rightTransferServo.setPower(autoNudgePower);
        } else {
            leftTransferServo.setPower(0);
            rightTransferServo.setPower(0);
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
