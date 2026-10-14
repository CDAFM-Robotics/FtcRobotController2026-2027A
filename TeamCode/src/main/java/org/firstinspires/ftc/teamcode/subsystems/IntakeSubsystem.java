package org.firstinspires.ftc.teamcode.subsystems;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.devices.actuators.MotorDevice;
import com.cdarobotics.cdalib.devices.actuators.ServoDevice;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class IntakeSubsystem extends Subsystem {

    private final MotorDevice intakeMotor;

    private final ServoDevice extenderServo1;
    private final ServoDevice extenderServo2;

    private BindingManager bindingManager;

    private final Telemetry telemetry;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry, String intakeMotorName, String extenderServoName1, String extenderServoName2) {
        intakeMotor = new MotorDevice(hardwareMap, intakeMotorName);
        extenderServo1 = new ServoDevice(hardwareMap, extenderServoName1);
        extenderServo2 = new ServoDevice(hardwareMap, extenderServoName2);
        this.telemetry = telemetry;
    }

    public void setBindings(BindingManager bindingManager, DoubleSupplier intakeSpeed, BooleanSupplier extender) {
        bindingManager.addAnalog("intakeSpeed", intakeSpeed);
        bindingManager.addBinding("extender", extender);


        this.bindingManager = bindingManager;
    }
    //----------------------EDIT THESE-----------------//
    static double EXTENDED_POSITION = 1.0;
    static double RETRACTED_POSITION = 0.5;
    //-------------------------------------------------//

    public static final double POSITION_EXTENSION_SERVO_EXTENDED_LEFT = 1-EXTENDED_POSITION;
    public static final double POSITION_EXTENSION_SERVO_EXTENDED_RIGHT = EXTENDED_POSITION;
    public static final double POSITION_EXTENSION_SERVO_RETRACTED_LEFT = 1-RETRACTED_POSITION;
    public static final double POSITION_EXTENSION_SERVO_RETRACTED_RIGHT = RETRACTED_POSITION;

    @Override
    public void init() {
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extenderServo1.setPosition(0.5);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        intakeMotor.setPower(bindingManager.checkAnalog("intakeSpeed"));
        intakeMotor.update();

        if (bindingManager.checkBinding("extender")) {

            extenderServo1.setPosition(POSITION_EXTENSION_SERVO_EXTENDED_LEFT);
            extenderServo2.setPosition(POSITION_EXTENSION_SERVO_EXTENDED_RIGHT);

        }
        else {
            extenderServo1.setPosition(POSITION_EXTENSION_SERVO_RETRACTED_LEFT);
            extenderServo2.setPosition(POSITION_EXTENSION_SERVO_RETRACTED_RIGHT);
        }

        extenderServo1.update();
        extenderServo2.update();
    }

    @Override
    public void stop() {

    }
}
