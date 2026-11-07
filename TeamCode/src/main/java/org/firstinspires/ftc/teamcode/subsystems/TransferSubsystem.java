package org.firstinspires.ftc.teamcode.subsystems;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.devices.actuators.CRServoDevice;
import com.cdarobotics.cdalib.devices.actuators.MotorDevice;
import com.cdarobotics.cdalib.devices.sensors.RevColorSensorV3Device;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.util.LoopProfiler;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TransferSubsystem extends Subsystem {

    private static final double TRANSFER_POWER_STEP = 0.1;
    private static final double TRANSFER_SERVO_POWER = 1;
    private static final double AUTO_NUDGE_POWER = 0.5; // power when color sensor triggers auto-nudge

    private double transferMotorPower = 1.0;

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

    public TransferSubsystem(HardwareMap hardwareMap, String transferMotorName, String leftTransferServoName, String rightTransferServoName){
        transferMotor = new MotorDevice(hardwareMap, transferMotorName);
        leftTransferServo = new CRServoDevice(hardwareMap, leftTransferServoName);
        rightTransferServo = new CRServoDevice(hardwareMap, rightTransferServoName);
        RevColorSensorV3 cs1 = hardwareMap.tryGet(RevColorSensorV3.class, "colorSensor1");
        colorSensor1 = (cs1 == null) ? null : new RevColorSensorV3Device(cs1).setGain(10);

        RevColorSensorV3 cs2 = hardwareMap.tryGet(RevColorSensorV3.class, "colorSensor2");
        colorSensor2 = (cs2 == null) ? null : new RevColorSensorV3Device(cs2);
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
        boolean elementDetected = hasGameElement();

        if (manualRun) {
            transferMotor.setPower(transferMotorPower);
        } else if (elementDetected) {
            transferMotor.setPower(AUTO_NUDGE_POWER);
        } else {
            transferMotor.setPower(0);
        }

        if (bindingManager.checkBinding("transferServo")) {
            leftTransferServo.setPower(TRANSFER_SERVO_POWER);
            rightTransferServo.setPower(TRANSFER_SERVO_POWER);
        } else if (elementDetected) {
            leftTransferServo.setPower(AUTO_NUDGE_POWER);
            rightTransferServo.setPower(AUTO_NUDGE_POWER);
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
