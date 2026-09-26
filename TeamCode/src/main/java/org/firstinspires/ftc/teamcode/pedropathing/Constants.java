package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Robot configuration for Pedro Pathing 3.0.
 *
 * <p>Pedro 3 builds a {@link Follower} from three explicit pieces — a drivetrain, a localizer and a
 * following algorithm — instead of the 2.x {@code FollowerBuilder}/{@code FollowerConstants}. Each
 * piece is configured below and assembled in {@link #createFollower(HardwareMap)}.
 *
 * <p>Nothing here is tuned yet. Run the AutoTune procedures registered in {@link Tuning} <b>in this
 * order</b>; each produces the values the next one needs:
 *
 * <ol>
 *   <li><b>Mecanum Tuner</b> → the four {@code *Direction} values in {@link #drivetrainConfig}.
 *   <li><b>Pinpoint Tuner</b> → the pod directions and offsets in {@link #localizerConfig}.
 *   <li><b>Foresight Tuner</b> → the whole of {@link #foresightConfig}, which is empty until then.
 *   <li><b>Tests</b> → verifies the result.
 * </ol>
 *
 * <p>Each tuner ends on a page of generated Java. Paste it over the matching block below.
 */
public class Constants {
    /**
     * Drivetrain: four-motor mecanum.
     *
     * <p>Motor names carried over from the previous drive-encoder config. The directions are the
     * conventional result for mirror-mounted motors (left side reversed) and are a
     * <b>placeholder</b> — the Mecanum Tuner spins each motor and tells you the real answer.
     */
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeftDriveMotor");
        c.frontRightName.set("frontRightDriveMotor");
        c.backLeftName.set("backLeftDriveMotor");
        c.backRightName.set("backRightDriveMotor");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    /**
     * Localizer: goBILDA Pinpoint with 4-bar odometry pods.
     *
     * <p>Replaces the 2.x drive-encoder localizer, which Pedro 3 removed. The pod offsets and
     * directions are <b>placeholders</b>: the Pinpoint Tuner derives all four by having you push the
     * robot forward, push it left, and spin it 180°, then emits a finished {@code PinpointConfig}
     * block to paste over this one. Until then, zero offsets mean heading changes will corrupt the
     * position estimate.
     *
     * <p>Set {@code name} to whatever the Pinpoint is configured as in the Robot Controller.
     */
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);

        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    /**
     * Following algorithm: Foresight. <b>Not tuned yet — this block is intentionally empty.</b>
     *
     * <p>Twelve of {@link ForesightConfig}'s variables are declared with no default (the two
     * translational controllers, heading feedback, brake, coast, the three brake-coefficient
     * matrices, both max achievable velocities, and both natural decelerations). Reading an unset
     * one throws {@code IllegalStateException}. Those twelve are exactly what the Foresight Tuner
     * measures — they are properties of this robot's mass, wheels and battery, so there is no
     * sensible default and nothing should be guessed.
     *
     * <p>Run the Foresight Tuner and paste its generated block in place of this lambda body.
     * {@link #createAlgorithm()} fails with a readable message until you do.
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // Paste the Foresight Tuner's output here.
    });

    public static Mecanum createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static PinpointLocalizer createLocalizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    /**
     * Builds the Foresight algorithm, checking up front that it has been tuned.
     *
     * <p>Without this check the first unset variable surfaces as a bare {@code IllegalStateException}
     * partway through following a path — no field name, no hint. The tuner writes all twelve required
     * values in one block, so probing one of them is enough to tell "never tuned" from "tuned".
     */
    public static Foresight createAlgorithm() {
        try {
            foresightConfig.maxAchievableForwardVelocity.get();
        } catch (IllegalStateException e) {
            throw new IllegalStateException(
                    "Foresight has not been tuned. Run the Foresight Tuner in AutoTune and paste "
                            + "its generated block into Constants.foresightConfig.", e);
        }
        return new Foresight(foresightConfig);
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(createLocalizer(hardwareMap), createDrivetrain(hardwareMap), createAlgorithm());
    }
}
