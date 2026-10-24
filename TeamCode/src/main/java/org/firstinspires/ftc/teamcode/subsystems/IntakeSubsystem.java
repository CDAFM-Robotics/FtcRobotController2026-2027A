package org.firstinspires.ftc.teamcode.subsystems;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.devices.actuators.MotorDevice;
import com.cdarobotics.cdalib.devices.actuators.ServoDevice;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeSubsystem extends Subsystem {

    private final MotorDevice intakeMotor;

    private final ServoDevice leftExtenderServo;
    private final ServoDevice rightExtenderServo;

    private BindingManager bindingManager;

    private final Telemetry telemetry;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String intakeMotorName, String leftExtenderServoName, String rightExtenderServoName) {
        intakeMotor = new MotorDevice(hardwareMap, intakeMotorName);
        leftExtenderServo = new ServoDevice(hardwareMap, leftExtenderServoName);
        rightExtenderServo = new ServoDevice(hardwareMap, rightExtenderServoName);
        this.telemetry = telemetry;
    }

    public void setBindings(BindingManager bindingManager, BooleanSupplier intake, BooleanSupplier extender) {
        bindingManager.addBinding("intake", intake);
        bindingManager.addBinding("extendIntake", extender);
        this.bindingManager = bindingManager;
    }

    public void setBindings(BindingManager bindingManager, DoubleSupplier intakeSpeed, BooleanSupplier extender) {
        bindingManager.addAnalog("intakeSpeed", intakeSpeed);
        bindingManager.addBinding("extendIntake", extender);
        this.bindingManager = bindingManager;
        useAnalogIntake = true;
    }

    private boolean useAnalogIntake = false;


    public static final double POSITION_EXTENSION_SERVO_INIT_LEFT = 0.505;
    public static final double POSITION_EXTENSION_SERVO_RETRACTED_LEFT = 0.53;
    public static final double POSITION_EXTENSION_SERVO_EXTENDED_LEFT = 0.505 + 0.173;

    public static final double POSITION_EXTENSION_SERVO_INIT_RIGHT = 0.495;
    public static final double POSITION_EXTENSION_SERVO_RETRACTED_RIGHT = 0.47;
    public static final double POSITION_EXTENSION_SERVO_EXTENDED_RIGHT = 0.495 - 0.173;

    @Override
    public void init() {
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftExtenderServo.setPosition(POSITION_EXTENSION_SERVO_INIT_LEFT);
        rightExtenderServo.setPosition(POSITION_EXTENSION_SERVO_INIT_RIGHT);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {

        if (useAnalogIntake) {
            intakeMotor.setPower(bindingManager.checkAnalog("intakeSpeed"));
        } else if (bindingManager.checkBinding("intake")) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }

        if (bindingManager.checkBinding("extendIntake")) {
            leftExtenderServo.setPosition(POSITION_EXTENSION_SERVO_EXTENDED_LEFT);
            rightExtenderServo.setPosition(POSITION_EXTENSION_SERVO_EXTENDED_RIGHT);
        }
        else {
            leftExtenderServo.setPosition(POSITION_EXTENSION_SERVO_RETRACTED_LEFT);
            rightExtenderServo.setPosition(POSITION_EXTENSION_SERVO_RETRACTED_RIGHT);
        }

        intakeMotor.update();

        leftExtenderServo.update();
        rightExtenderServo.update();
    }

    @Override
    public void stop() {

    }
}
