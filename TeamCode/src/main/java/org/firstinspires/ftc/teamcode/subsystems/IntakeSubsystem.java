package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.devices.actuators.MotorDevice;
import com.cdarobotics.cdalib.devices.actuators.ServoDevice;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.LoopProfiler;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Configurable
public class IntakeSubsystem extends Subsystem {

    // --- Flower bar ---
    // A goBILDA Speed servo ("flowerBarServo", Control Hub servo port 5) swings a bar down to push Pollen out of the
    // Flower so the intake can pick them up. Optional: if the servo is not in the configuration, Flower pickup does
    // nothing and the rest of the intake works as before. Higher position = further down; the up hard stop is near 0.30.
    public static double flowerBarUpPosition   = 0.47;   // measured Nov 14 with FlowerBarServoTest
    public static double flowerBarDownPosition = 1.00;   // measured Nov 14
    public static int    flowerBarMoveMs       = 400;    // down -> up travel; extender stays in this long after a raise
    public static double flowerIntakePower     = 1.0;    // intake power while the bar is down (0.75 felt slow, Nov 14)
    public static double flowerIntakeStopDelaySec = 0.5; // intake keeps running this long after the raise starts

    private final MotorDevice intakeMotor;
    private final ServoDevice leftExtenderServo;
    private final ServoDevice rightExtenderServo;

    // Axon feedback — null if not wired
    private final AnalogInput leftFeedback;
    private final AnalogInput rightFeedback;

    // Extender feedback voltages, measured Nov 14 with ExtenderFeedbackCalibration (old left values were 1.885 / 2.196).
    // INIT for reference: left 1.670, right 1.635. Left rises as the extender goes out; right falls (mirrored servo).
    // Resting noise is about +/-0.004 V; readings spike while the servos move, hence the hold time below.
    public static double extLeftRetractedV  = 1.717;
    public static double extLeftExtendedV   = 2.163;
    public static double extRightRetractedV = 1.568;
    public static double extRightExtendedV  = 1.130;
    // One-sided thresholds: "retracted" = at RETRACTED or further in (INIT counts), within this margin;
    // "extended" = at EXTENDED or further out, within this margin.
    public static double extFeedbackMarginV = 0.03;
    // A side's reading must satisfy the threshold continuously this long before it counts (filters motion spikes).
    public static int    extFeedbackHoldMs  = 100;

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

    // Debounced extender state, computed once per update() from both feedback signals.
    private boolean retractedStable = false;
    private boolean extendedStable  = false;
    private final ElapsedTime retractedRawSince = new ElapsedTime();
    private final ElapsedTime extendedRawSince  = new ElapsedTime();
    private boolean retractedRawPrev = false;
    private boolean extendedRawPrev  = false;

    private final Servo flowerBarServo;            // null if not configured
    private boolean flowerBound = false;           // setFlowerBinding() called
    private boolean flowerBarDown = false;         // bar commanded down
    private boolean flowerRaisedOnce = false;      // a raise has happened (so the stop-delay tail can run)
    private final ElapsedTime sinceFlowerRaise = new ElapsedTime();

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry,
                           String intakeMotorName,
                           String leftExtenderServoName,
                           String rightExtenderServoName) {
        intakeMotor        = new MotorDevice(hardwareMap, intakeMotorName);
        leftExtenderServo  = new ServoDevice(hardwareMap, leftExtenderServoName);
        rightExtenderServo = new ServoDevice(hardwareMap, rightExtenderServoName);
        leftFeedback  = hardwareMap.tryGet(AnalogInput.class, "intakeLeftServoSignal");
        rightFeedback = hardwareMap.tryGet(AnalogInput.class, "intakeRightServoSignal");
        flowerBarServo = hardwareMap.tryGet(Servo.class, "flowerBarServo");
        this.telemetry = telemetry;
    }

    /**
     * Flower pickup: while flowerPickup is true, the extender retracts, then the bar lowers and the intake runs at
     * flowerIntakePower. When it goes false, the bar raises and the intake stops flowerIntakeStopDelaySec later. The
     * intake binding (RT/LT in TeleOp) overrides the motor whenever it is non-zero. The extender cannot extend while
     * Flower pickup is requested, the bar is down, or the bar is still rising.
     */
    public void setFlowerBinding(BindingManager bindingManager, BooleanSupplier flowerPickup) {
        bindingManager.addBinding("flowerPickup", flowerPickup);
        flowerBound = true;
    }

    public boolean isFlowerBarConfigured() { return flowerBarServo != null; }
    public boolean isFlowerBarDown()       { return flowerBarDown; }

    /** Flower pickup requested but the bar is still up, waiting for the extender to retract. */
    public boolean isFlowerWaitingForRetract() {
        return flowerBound && bindingManager.checkBinding("flowerPickup") && !flowerBarDown && flowerBarServo != null;
    }

    /** Bar is up and has had flowerBarMoveMs to get there since the last raise. */
    public boolean isFlowerBarUp() {
        return !flowerBarDown && (!flowerRaisedOnce || sinceFlowerRaise.milliseconds() >= flowerBarMoveMs);
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

    /** Extender at EXTENDED or further out: every wired feedback side agrees and has held for extFeedbackHoldMs.
     *  Updated once per update(); false before the first update(). */
    public boolean isExtended() {
        return extendedStable;
    }

    /** Extender at RETRACTED or further in (INIT counts): every wired feedback side agrees and has held for
     *  extFeedbackHoldMs. Updated once per update(); false before the first update(). */
    public boolean isRetracted() {
        return retractedStable;
    }

    /** True if voltage v is at 'end' or beyond it (away from 'other'), allowing margin. Works for rising or falling sides. */
    private static boolean atOrBeyond(double v, double end, double other, double margin) {
        double outward = Math.signum(end - other);   // direction from the other end toward this end
        return (v - end) * outward >= -margin;
    }

    /** Reads both feedback signals once and updates the debounced retracted/extended state. */
    private void updateExtenderFeedback() {
        double vL = leftFeedback  != null ? leftFeedback.getVoltage()  : Double.NaN;
        double vR = rightFeedback != null ? rightFeedback.getVoltage() : Double.NaN;

        boolean retractedRaw, extendedRaw;
        if (leftFeedback == null && rightFeedback == null) {
            // No feedback wired: fall back to the commanded position estimate.
            retractedRaw = Math.abs(currentLeftPos - POSITION_EXTENSION_SERVO_RETRACTED_LEFT) < SLOW_ZONE / 2;
            extendedRaw  = Math.abs(currentLeftPos - POSITION_EXTENSION_SERVO_EXTENDED_LEFT)  < SLOW_ZONE / 2;
        } else {
            retractedRaw = (leftFeedback  == null || atOrBeyond(vL, extLeftRetractedV,  extLeftExtendedV,   extFeedbackMarginV))
                        && (rightFeedback == null || atOrBeyond(vR, extRightRetractedV, extRightExtendedV,  extFeedbackMarginV));
            extendedRaw  = (leftFeedback  == null || atOrBeyond(vL, extLeftExtendedV,   extLeftRetractedV,  extFeedbackMarginV))
                        && (rightFeedback == null || atOrBeyond(vR, extRightExtendedV,  extRightRetractedV, extFeedbackMarginV));
        }

        if (retractedRaw && !retractedRawPrev) retractedRawSince.reset();
        if (extendedRaw  && !extendedRawPrev)  extendedRawSince.reset();
        retractedRawPrev = retractedRaw;
        extendedRawPrev  = extendedRaw;
        retractedStable = retractedRaw && retractedRawSince.milliseconds() >= extFeedbackHoldMs;
        extendedStable  = extendedRaw  && extendedRawSince.milliseconds()  >= extFeedbackHoldMs;
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
        // Bar up at init.
        flowerBarDown = false;
        flowerRaisedOnce = false;
        if (flowerBarServo != null) flowerBarServo.setPosition(flowerBarUpPosition);
    }

    @Override public void init_loop() {}
    @Override public void start() {}

    @Override
    public void update() {
        updateExtenderFeedback();

        // Flower bar: lower only once the extender is retracted; raise as soon as pickup is no longer requested.
        boolean flowerWanted = flowerBound && bindingManager.checkBinding("flowerPickup");
        if (flowerWanted && !flowerBarDown && flowerBarServo != null && isRetracted()) {
            flowerBarDown = true;
        } else if (!flowerWanted && flowerBarDown) {
            flowerBarDown = false;
            flowerRaisedOnce = true;
            sinceFlowerRaise.reset();
        }
        if (flowerBarServo != null) {
            flowerBarServo.setPosition(flowerBarDown ? flowerBarDownPosition : flowerBarUpPosition);
        }
        boolean flowerIntakeOn = flowerBarDown
                || (flowerRaisedOnce && sinceFlowerRaise.seconds() < flowerIntakeStopDelaySec);

        // Intake motor: the driver's binding wins whenever it asks for anything; otherwise Flower pickup runs it.
        double requested;
        if (useAnalogIntake) {
            requested = bindingManager.checkAnalog("intakeSpeed");
        } else {
            requested = bindingManager.checkBinding("intake") ? 1 : 0;
        }
        intakeMotor.setPower(requested != 0 ? requested : (flowerIntakeOn ? flowerIntakePower : 0));

        // Smooth servo ramp toward target. Interlock: the extender stays retracted while Flower pickup is requested
        // (this is what retracts it before the bar lowers), while the bar is down, and while the bar is still rising.
        boolean extend = bindingManager.checkBinding("extendIntake")
                && !flowerWanted && isFlowerBarUp();
        double leftGoal  = extend
                ? POSITION_EXTENSION_SERVO_EXTENDED_LEFT
                : POSITION_EXTENSION_SERVO_RETRACTED_LEFT;
        double rightGoal = extend
                ? POSITION_EXTENSION_SERVO_EXTENDED_RIGHT
                : POSITION_EXTENSION_SERVO_RETRACTED_RIGHT;

        currentLeftPos  = stepToward(currentLeftPos,  leftGoal);
        currentRightPos = stepToward(currentRightPos, rightGoal);

        leftExtenderServo.setPosition(currentLeftPos);
        rightExtenderServo.setPosition(currentRightPos);

        intakeMotor.update();
        leftExtenderServo.update();
        rightExtenderServo.update();
        LoopProfiler.mark("intake");
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
