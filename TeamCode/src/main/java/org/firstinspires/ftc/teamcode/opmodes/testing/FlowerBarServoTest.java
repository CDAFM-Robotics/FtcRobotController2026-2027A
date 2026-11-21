package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Finds the Flower bar's UP and DOWN positions. Moves only flowerBarServo (goBILDA Speed servo, Control Hub servo
 * port 5). The servo is not commanded during init; it goes to START_POSITION when Start is pressed and is clamped to
 * [MIN_POSITION, MAX_POSITION].
 *
 * gamepad1: dpad up/down +/-0.01 | RB/LB +/-0.05 | A save DOWN | Y save UP | X go to DOWN | B go to UP
 */
@Disabled   // Nov 20: hidden for competition; remove this line to use it
@TeleOp(name = "Flower Bar Servo Test", group = "Testing")
public class FlowerBarServoTest extends LinearOpMode {

    private static final double START_POSITION = 0.50;   // mid-range; at 0.30 the bar was driven into its up hard stop
    private static final double MIN_POSITION   = 0.00;
    private static final double MAX_POSITION   = 1.00;   // full range: direction of 'up' not yet known
    private static final double FINE_STEP      = 0.01;
    private static final double COARSE_STEP    = 0.05;

    // goBILDA Speed servo: ~300 deg over the full 0-1 range, ~0.055 s per 60 deg at 6 V with no load. Used only for a
    // rough travel-time estimate; the servo has no feedback, so the real time must be checked by eye.
    private static final double DEGREES_PER_UNIT = 300.0;
    private static final double SEC_PER_60_DEG   = 0.055;

    @Override
    public void runOpMode() {
        Servo bar = hardwareMap.tryGet(Servo.class, "flowerBarServo");

        while (opModeInInit()) {
            telemetry.addData("flowerBarServo", bar == null ? "NOT FOUND in robot configuration" : "found");
            telemetry.addLine("The servo will not move until Start. It then goes to " + START_POSITION + ".");
            telemetry.update();
        }
        if (bar == null) return;

        double position = START_POSITION;
        double savedDown = Double.NaN;
        double savedUp = Double.NaN;
        bar.setPosition(position);

        boolean prevUp = false, prevDown = false, prevRB = false, prevLB = false;
        boolean prevA = false, prevY = false, prevX = false, prevB = false;

        while (opModeIsActive()) {
            double target = position;

            if (gamepad1.dpad_up && !prevUp)          target += FINE_STEP;
            if (gamepad1.dpad_down && !prevDown)      target -= FINE_STEP;
            if (gamepad1.right_bumper && !prevRB)     target += COARSE_STEP;
            if (gamepad1.left_bumper && !prevLB)      target -= COARSE_STEP;
            if (gamepad1.a && !prevA)                 savedDown = position;
            if (gamepad1.y && !prevY)                 savedUp = position;
            if (gamepad1.x && !prevX && !Double.isNaN(savedDown)) target = savedDown;
            if (gamepad1.b && !prevB && !Double.isNaN(savedUp))   target = savedUp;

            prevUp = gamepad1.dpad_up;     prevDown = gamepad1.dpad_down;
            prevRB = gamepad1.right_bumper; prevLB = gamepad1.left_bumper;
            prevA = gamepad1.a; prevY = gamepad1.y; prevX = gamepad1.x; prevB = gamepad1.b;

            // Round to 0.01 so repeated steps do not drift (0.30 - 0.01 * 3 stays 0.27).
            position = Math.round(Math.max(MIN_POSITION, Math.min(MAX_POSITION, target)) * 100.0) / 100.0;
            bar.setPosition(position);

            telemetry.addData("Position", "%.2f", position);
            telemetry.addData("Saved DOWN (A)", Double.isNaN(savedDown) ? "-" : String.format("%.2f", savedDown));
            telemetry.addData("Saved UP (Y)", Double.isNaN(savedUp) ? "-" : String.format("%.2f", savedUp));
            if (!Double.isNaN(savedDown) && !Double.isNaN(savedUp)) {
                double degrees = Math.abs(savedUp - savedDown) * DEGREES_PER_UNIT;
                double estMs = degrees / 60.0 * SEC_PER_60_DEG * 1000.0;
                telemetry.addData("DOWN -> UP travel", "%.0f deg, ~%.0f ms unloaded (check by eye)", degrees, estMs);
            }
            telemetry.addLine("dpad up/down: +/-0.01 | RB/LB: +/-0.05 | A: save DOWN | Y: save UP | X: go DOWN | B: go UP");
            telemetry.addLine("Allowed range " + MIN_POSITION + " - " + MAX_POSITION);
            telemetry.update();
        }
    }
}
