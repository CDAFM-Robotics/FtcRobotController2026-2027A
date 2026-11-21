package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Measures the intake extender's Axon feedback voltages (left and right) at INIT, RETRACTED and EXTENDED, so
 * IntakeSubsystem.isRetracted()/isExtended() can be calibrated. Moves only the two extender servos, with the same
 * ramp as IntakeSubsystem. Nothing moves during init; on Start the extender is set to INIT, as IntakeSubsystem does.
 *
 * Log tag "ExtenderFeedback": a "live" line every LIVE_LOG_PERIOD_MS (target, commanded positions, settled, voltages)
 * and a "recorded" line for each RB recording, so the numbers can be pulled with adb logcat -s ExtenderFeedback.
 *
 * gamepad1: X go INIT | A go RETRACTED | Y go EXTENDED | RB record the current position (0.5 s average)
 */
@Disabled   // Nov 20: hidden for competition; remove this line to use it
@TeleOp(name = "Extender Feedback Calibration", group = "Testing")
public class ExtenderFeedbackCalibration extends LinearOpMode {

    // Same ramp as IntakeSubsystem (private there).
    private static final double SERVO_STEP_FAST = 0.010;
    private static final double SERVO_STEP_SLOW = 0.003;
    private static final double SLOW_ZONE       = 0.05;

    private static final double SETTLE_SEC = 0.5;   // wait at the goal before a recording is allowed
    private static final double RECORD_SEC = 0.5;   // averaging window
    private static final int LIVE_LOG_PERIOD_MS = 100;
    private static final String TAG = "ExtenderFeedback";

    private static final String[] NAMES = {"INIT", "RETRACTED", "EXTENDED"};
    private static final double[] LEFT_POS = {
            IntakeSubsystem.POSITION_EXTENSION_SERVO_INIT_LEFT,
            IntakeSubsystem.POSITION_EXTENSION_SERVO_RETRACTED_LEFT,
            IntakeSubsystem.POSITION_EXTENSION_SERVO_EXTENDED_LEFT};
    private static final double[] RIGHT_POS = {
            IntakeSubsystem.POSITION_EXTENSION_SERVO_INIT_RIGHT,
            IntakeSubsystem.POSITION_EXTENSION_SERVO_RETRACTED_RIGHT,
            IntakeSubsystem.POSITION_EXTENSION_SERVO_EXTENDED_RIGHT};

    @Override
    public void runOpMode() {
        Servo leftServo  = hardwareMap.get(Servo.class, "leftExtenderServo");
        Servo rightServo = hardwareMap.get(Servo.class, "rightExtenderServo");
        AnalogInput leftFb  = hardwareMap.tryGet(AnalogInput.class, "intakeLeftServoSignal");
        AnalogInput rightFb = hardwareMap.tryGet(AnalogInput.class, "intakeRightServoSignal");

        while (opModeInInit()) {
            telemetry.addData("Left feedback (intakeLeftServoSignal)", leftFb == null ? "NOT FOUND" : String.format("%.3f V", leftFb.getVoltage()));
            telemetry.addData("Right feedback (intakeRightServoSignal)", rightFb == null ? "NOT FOUND" : String.format("%.3f V", rightFb.getVoltage()));
            telemetry.addLine("Nothing moves until Start. On Start the extender goes to INIT.");
            telemetry.update();
        }

        int target = 0;                       // index into NAMES
        double curL = LEFT_POS[0], curR = RIGHT_POS[0];
        leftServo.setPosition(curL);
        rightServo.setPosition(curR);

        double[] recL = {Double.NaN, Double.NaN, Double.NaN};
        double[] recR = {Double.NaN, Double.NaN, Double.NaN};

        ElapsedTime atGoal = new ElapsedTime();
        boolean wasAtGoal = false;
        boolean recording = false;
        ElapsedTime recordTimer = new ElapsedTime();
        double sumL = 0, sumR = 0;
        int samples = 0;
        boolean prevX = false, prevA = false, prevY = false, prevRB = false;
        ElapsedTime liveLogTimer = new ElapsedTime();

        while (opModeIsActive()) {
            int newTarget = target;
            if (gamepad1.x && !prevX) newTarget = 0;
            if (gamepad1.a && !prevA) newTarget = 1;
            if (gamepad1.y && !prevY) newTarget = 2;
            if (newTarget != target) { target = newTarget; recording = false; }

            curL = stepToward(curL, LEFT_POS[target]);
            curR = stepToward(curR, RIGHT_POS[target]);
            leftServo.setPosition(curL);
            rightServo.setPosition(curR);

            boolean isAtGoal = curL == LEFT_POS[target] && curR == RIGHT_POS[target];
            if (isAtGoal && !wasAtGoal) atGoal.reset();
            wasAtGoal = isAtGoal;
            boolean settled = isAtGoal && atGoal.seconds() >= SETTLE_SEC;

            double vL = leftFb  == null ? Double.NaN : leftFb.getVoltage();
            double vR = rightFb == null ? Double.NaN : rightFb.getVoltage();

            if (gamepad1.right_bumper && !prevRB && settled && !recording) {
                recording = true;
                recordTimer.reset();
                sumL = 0; sumR = 0; samples = 0;
            }
            if (recording) {
                sumL += vL; sumR += vR; samples++;
                if (recordTimer.seconds() >= RECORD_SEC) {
                    recL[target] = sumL / samples;
                    recR[target] = sumR / samples;
                    recording = false;
                    Log.d(TAG, String.format("recorded pos=%s cmdL=%.3f cmdR=%.3f avgL=%.4f avgR=%.4f samples=%d",
                            NAMES[target], LEFT_POS[target], RIGHT_POS[target], recL[target], recR[target], samples));
                }
            }

            if (liveLogTimer.milliseconds() >= LIVE_LOG_PERIOD_MS) {
                liveLogTimer.reset();
                Log.d(TAG, String.format("live target=%s cmdL=%.3f cmdR=%.3f settled=%d vL=%.4f vR=%.4f",
                        NAMES[target], curL, curR, settled ? 1 : 0, vL, vR));
            }

            prevX = gamepad1.x; prevA = gamepad1.a; prevY = gamepad1.y; prevRB = gamepad1.right_bumper;

            telemetry.addData("Target", "%s  (L %.3f / R %.3f)", NAMES[target], LEFT_POS[target], RIGHT_POS[target]);
            telemetry.addData("Status", recording ? "RECORDING..." : (settled ? "settled - RB to record" : "moving / settling"));
            telemetry.addData("Live feedback", "L %.3f V   R %.3f V", vL, vR);
            for (int i = 0; i < 3; i++) {
                telemetry.addData("Recorded " + NAMES[i], Double.isNaN(recL[i]) ? "-" : String.format("L %.3f V   R %.3f V", recL[i], recR[i]));
            }
            telemetry.addData("Gap INIT-RETRACTED", gapText(recL, recR, 0, 1));
            telemetry.addData("Gap RETRACTED-EXTENDED", gapText(recL, recR, 1, 2));
            telemetry.addData("Suggested tolerance", toleranceText(recL, recR));
            telemetry.addLine("X: INIT | A: RETRACTED | Y: EXTENDED | RB: record (after settled)");
            telemetry.update();
        }
    }

    private static String gapText(double[] l, double[] r, int a, int b) {
        if (Double.isNaN(l[a]) || Double.isNaN(l[b])) return "-";
        return String.format("L %.3f V   R %.3f V", Math.abs(l[a] - l[b]), Math.abs(r[a] - r[b]));
    }

    /** Half the smallest gap between any two recorded positions, per side. */
    private static String toleranceText(double[] l, double[] r) {
        double minL = Double.MAX_VALUE, minR = Double.MAX_VALUE;
        int pairs = 0;
        for (int i = 0; i < 3; i++) {
            for (int j = i + 1; j < 3; j++) {
                if (Double.isNaN(l[i]) || Double.isNaN(l[j])) continue;
                minL = Math.min(minL, Math.abs(l[i] - l[j]));
                minR = Math.min(minR, Math.abs(r[i] - r[j]));
                pairs++;
            }
        }
        if (pairs == 0) return "record at least two positions";
        return String.format("L +/-%.3f V   R +/-%.3f V", minL / 2, minR / 2);
    }

    // Same as IntakeSubsystem.stepToward.
    private static double stepToward(double current, double goal) {
        double delta = goal - current;
        if (Math.abs(delta) < 0.001) return goal;
        double step = Math.abs(delta) < SLOW_ZONE ? SERVO_STEP_SLOW : SERVO_STEP_FAST;
        return current + Math.signum(delta) * Math.min(Math.abs(delta), step);
    }
}
