package org.firstinspires.ftc.teamcode.subsystems;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.devices.actuators.CRServoDevice;
import com.cdarobotics.cdalib.devices.actuators.MotorDevice;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TransferSubsystem extends Subsystem {

    private static final double TRANSFER_POWER_STEP = 0.1;
    private static final double TRANSFER_SERVO_POWER = 1;
    private static final double AUTO_NUDGE_POWER = 0.3; // power when color sensor triggers auto-nudge

    private double transferMotorPower = 1.0;

    private final MotorDevice transferMotor;
    private final CRServoDevice leftTransferServo;
    private final CRServoDevice rightTransferServo;
    private final NormalizedColorSensor colorSensor;
    private final DistanceSensor distanceSensor;
    private static final double GAME_ELEMENT_DISTANCE_CM = 5.0;

    private BindingManager bindingManager;

    public TransferSubsystem(HardwareMap hardwareMap, String transferMotorName, String leftTransferServoName, String rightTransferServoName){
        transferMotor = new MotorDevice(hardwareMap, transferMotorName);
        leftTransferServo = new CRServoDevice(hardwareMap, leftTransferServoName);
        rightTransferServo = new CRServoDevice(hardwareMap, rightTransferServoName);
        colorSensor = hardwareMap.tryGet(NormalizedColorSensor.class, "colorSensor1");
        if (colorSensor != null) colorSensor.setGain(10);
        distanceSensor = (colorSensor instanceof DistanceSensor) ? (DistanceSensor) colorSensor : null;
    }

    public boolean hasGameElement() {
        if (distanceSensor == null) return false;
        double distance = distanceSensor.getDistance(DistanceUnit.CM);
        return distance < GAME_ELEMENT_DISTANCE_CM;
    }

    public double getDistance() {
        if (distanceSensor == null) return Double.NaN;
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    public NormalizedRGBA getColorReading() {
        if (colorSensor == null) return null;
        return colorSensor.getNormalizedColors();
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
    }

    @Override
    public void stop() {

    }
}
