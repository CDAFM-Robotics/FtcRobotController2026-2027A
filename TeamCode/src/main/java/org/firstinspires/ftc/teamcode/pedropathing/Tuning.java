package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedropathing.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedropathing.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedropathing.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedropathing.procedures.Tests;

/**
 * The procedures Pedro Pathing 3.0's AutoTune offers on its web page.
 *
 * <p>This replaces the 2.x {@code SelectableOpMode}-based tuners (ForwardTuner, LateralTuner,
 * CentripetalTuner, …), which Pedro 3 removed along with the PID follower they tuned. AutoTune's
 * {@code TunerScanner} finds the entries below by reflection at OpMode-discovery time: each must be
 * a <b>static, no-argument method returning {@link Procedure}</b>, or startup fails with an
 * {@code IllegalArgumentException} naming the offending method.
 *
 * <p>Only the procedures matching this robot's hardware are registered — mecanum drivetrain,
 * Pinpoint localizer. The Quickstart also ships OTOS, OctoQuad, two-wheel and three-wheel tuners; if
 * the localizer changes, copy that procedure into {@code procedures/} and add a {@code @Tuner} entry.
 *
 * <p>See {@link Constants} for the order to run these in and where each one's output goes.
 */
public class Tuning {
    @Tuner(name = "Mecanum Tuner")
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner(name = "Pinpoint Tuner")
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner(name = "Foresight Tuner")
    public static Procedure foresightTuner() {
        return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
    }

    /**
     * The localization, odometry, pose and driving tests need only the drivetrain and localizer, so
     * they are usable before Foresight is tuned. The hold, line and curve tests build a
     * {@link com.pedropathing.follower.Follower} and will report Foresight as untuned until it is.
     */
    @Tuner(name = "Tests")
    public static Procedure tests() {
        return new Tests(Constants::createDrivetrain, Constants::createLocalizer, Constants::createAlgorithm);
    }
}
