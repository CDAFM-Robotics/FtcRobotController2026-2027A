package org.firstinspires.ftc.teamcode.subsystems;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.devices.actuators.MotorDevice;
import com.cdarobotics.cdalib.devices.actuators.ServoDevice;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.AnalogInput;
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

    // Axon feedback — null if not wired
    private final AnalogInput leftFeedback;
    private final AnalogInput rightFeedback;

    private static final double FEEDBACK_RETRACTED_VOLTAGE = 1.885;
    private static final double FEEDBACK_EXTENDED_VOLTAGE  = 2.196;
    private static final double FEEDBACK_TOLERANCE         = 0.05; // ±V to count as "at position"

    private BindingManager bindingManager;
    private final Telemetry telemetry;
    private boolean useAnalogIntake = false;

    // Servo position constants — DO NOT CHANGE
    public static final double POSITION_EXTENSION_SERVO_INIT_LEFT      = 0.505;
    public static final double POSITION_EXTENSION_SERVO_RETRACTED_LEFT = 0.525;
    public static final double POSITION_EXTENSION_SERVO_EXTENDED_LEFT  = 0.505 + 0.173;

    public static final double POSITION_EXTENSION_SERVO_INIT_RIGHT      = 0.495;
    public static final double POSITION_EXTENSION_SERVO_RETRACTED_RIGHT = 0.475;
    public static final double POSITION_EXTENSION_SERVO_EXTENDED_RIGHT  = 0.495 - 0.173;

    // Smooth ramp: fast step far from target, slow step near target
    private static final double SERVO_STEP_FAST  = 0.010; // per loop iteration
    private static final double SERVO_STEP_SLOW  = 0.003;
    private static final double SLOW_ZONE        = 0.05;  // within this distance → slow

    private double currentLeftPos  = POSITION_EXTENSION_SERVO_INIT_LEFT;
    private double currentRightPos = POSITION_EXTENSION_SERVO_INIT_RIGHT;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry,
                           String intakeMotorName,
                           String leftExtenderServoName,
                           String rightExtenderServoName) {
        intakeMotor        = new MotorDevice(hardwareMap, intakeMotorName);
        leftExtenderServo  = new ServoDevice(hardwareMap, leftExtenderServoName);
        rightExtenderServo = new ServoDevice(hardwareMap, rightExtenderServoName);
        leftFeedback  = hardwareMap.tryGet(AnalogInput.class, "intakeLeftServoSignal");
        rightFeedback = hardwareMap.tryGet(AnalogInput.class, "intakeRightServoSignal");
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

    // --- Feedback accessors ---

    public double getLeftFeedbackVoltage() {
        return leftFeedback != null ? leftFeedback.getVoltage() : Double.NaN;
    }

    public double getRightFeedbackVoltage() {
        return rightFeedback != null ? rightFeedback.getVoltage() : Double.NaN;
    }

    public boolean isExtended() {
        if (leftFeedback == null) {
            // Fall back to position estimate when no feedback wired
            return Math.abs(currentLeftPos - POSITION_EXTENSION_SERVO_EXTENDED_LEFT) < SLOW_ZONE / 2;
        }
        return Math.abs(leftFeedback.getVoltage() - FEEDBACK_EXTENDED_VOLTAGE) < FEEDBACK_TOLERANCE;
    }

    public boolean isRetracted() {
        if (leftFeedback == null) {
            return Math.abs(currentLeftPos - POSITION_EXTENSION_SERVO_RETRACTED_LEFT) < SLOW_ZONE / 2;
        }
        return Math.abs(leftFeedback.getVoltage() - FEEDBACK_RETRACTED_VOLTAGE) < FEEDBACK_TOLERANCE;
    }

    // --- Subsystem lifecycle ---

    @Override
    public void init() {
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        currentLeftPos  = POSITION_EXTENSION_SERVO_INIT_LEFT;
        currentRightPos = POSITION_EXTENSION_SERVO_INIT_RIGHT;
        leftExtenderServo.setPosition(currentLeftPos);
        rightExtenderServo.setPosition(currentRightPos);
        leftExtenderServo.update();
        rightExtenderServo.update();
    }

    @Override public void init_loop() {}
    @Override public void start() {}

    @Override
    public void update() {
        // Intake motor
        if (useAnalogIntake) {
            intakeMotor.setPower(bindingManager.checkAnalog("intakeSpeed"));
        } else if (bindingManager.checkBinding("intake")) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }

        // Smooth servo ramp toward target
        double leftGoal  = bindingManager.checkBinding("extendIntake")
                ? POSITION_EXTENSION_SERVO_EXTENDED_LEFT
                : POSITION_EXTENSION_SERVO_RETRACTED_LEFT;
        double rightGoal = bindingManager.checkBinding("extendIntake")
                ? POSITION_EXTENSION_SERVO_EXTENDED_RIGHT
                : POSITION_EXTENSION_SERVO_RETRACTED_RIGHT;

        currentLeftPos  = stepToward(currentLeftPos,  leftGoal);
        currentRightPos = stepToward(currentRightPos, rightGoal);

        leftExtenderServo.setPosition(currentLeftPos);
        rightExtenderServo.setPosition(currentRightPos);

        intakeMotor.update();
        leftExtenderServo.update();
        rightExtenderServo.update();
    }

    @Override public void stop() {}

    // Moves value toward goal with fast step when far, slow step near end
    private static double stepToward(double current, double goal) {
        double delta = goal - current;
        if (Math.abs(delta) < 0.001) return goal;
        double step = Math.abs(delta) < SLOW_ZONE ? SERVO_STEP_SLOW : SERVO_STEP_FAST;
        return current + Math.signum(delta) * Math.min(Math.abs(delta), step);
    }
}
