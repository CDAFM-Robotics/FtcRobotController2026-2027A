package org.firstinspires.ftc.teamcode.opmodes.testing;

import static com.pedropathing.api.Paths.line;

import com.bylazar.configurables.annotations.Configurable;
import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.cdarobotics.cdalib.tasks.DeferredTask;
import com.cdarobotics.cdalib.tasks.FollowPathTask;
import com.cdarobotics.cdalib.tasks.NullTask;
import com.cdarobotics.cdalib.tasks.RaceTask;
import com.cdarobotics.cdalib.tasks.WaitUntilTask;
import com.cdarobotics.cdalib.tasks.SequentialTask;
import com.cdarobotics.cdalib.tasks.SleepTask;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.Paths;
import org.firstinspires.ftc.teamcode.subsystems.AutoAimSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FollowerPoseSource;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.tasks.Tasks;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.PoseStorage.Alliance;

/**
 * TEST: Red Garden + pick up the 4 Pollen in the Red Garden corner.
 *
 * 1. Shoot the 4 preloads from the Red Garden start spot (same as Red Garden).
 * 2. Turn to heading 180 (facing the red wall) and move to the sweep start beside the y = 0 wall.
 * 3. Intake on, drive SLOWLY along the wall into the corner (heading held at 180; intake retracted unless
 *    extendIntake). The 4 Pollen sit in a row against the y = 0 wall from the corner (centers y ~1.4, x ~1.4-9.8).
 * 4. Keep the intake running intakeDwellMs, then stop it.
 * 5. Back to the shoot spot and check the HIVE with Webcam 1 (HiveTipDetector, Red audience CELL tags 34-37,
 *    baseline recorded during INIT from this same spot):
 *      - Red audience CELL still UP  -> shoot the picked-up balls, then park.
 *      - Red audience CELL TIPPED    -> keep the balls, park.
 *      - can't tell (no camera / no tags within hiveCheckMs) -> shoot if shootIfHiveUnknown, else keep them.
 *    Park is the same as Red Garden.
 *
 * The sweep spots and speed are Panels values (RedGardenPickupTest), read when the OpMode is INIT-ed, so a change
 * takes effect on the next INIT without rebuilding. Robot is 18 x 18 with the tracking center in the middle, so
 * y = 9 puts its side on the y = 0 wall and x = 9 puts its front bumper on the red wall.
 */
@Configurable
@Autonomous(name = "Red Garden Pickup TEST", group = "Testing")
public class RedGardenPickupTest extends ModularOpMode {

    public static double sweepY = 9.5;          // tracking center y along the wall (9 = side touching the wall)
    public static double sweepStartX = 32.0;    // where the slow sweep starts
    public static double sweepEndX = 10.0;      // where it stops (9 = front bumper on the red wall)
    public static double sweepSpeed = 0.3;      // fraction of full speed for the sweep only
    public static long intakeDwellMs = 1000;    // keep the intake on this long after the sweep
    public static boolean shootPickedUp = true; // go back and shoot the picked-up balls before parking
    public static boolean extendIntake = false; // extend the intake for the sweep (then lower sweepEndX's reach:
                                                // stop before the extended intake hits the red wall)
    public static double hiveTipThresholdDeg = 20; // tag pitch change that counts as tipped (a tip is ~60 deg)
    public static long hiveCheckMs = 1000;      // how long to look at the HIVE from the shoot spot
    public static boolean shootIfHiveUnknown = true; // camera can't tell: shoot anyway (true) or keep the balls

    private static final int[] RED_AUDIENCE_CELL_TAGS = {34, 35, 36, 37};
    private HiveTipDetector hive;
    private String hiveDecision = "not checked yet";

    private Follower follower;
    private Tasks tasks;

    @Override
    protected void preload() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(Paths.POSE_SHOOT_RED_GARDEN);   // re-applied in start() after the Pinpoint IMU recalibration

        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, "intakeMotor", "leftExtenderServo", "rightExtenderServo");
        TransferSubsystem transferSubsystem = new TransferSubsystem(hardwareMap, "transferMotor", "leftTransferServo", "rightTransferServo");
        LauncherSubsystem launcherSubsystem = new LauncherSubsystem(hardwareMap, telemetry, "pollenLauncherMotor", "nectarLauncherMotor");

        // Same launcher speeds as Red Garden: distance table + 90.
        AutoAimSubsystem autoAim = new AutoAimSubsystem(new FollowerPoseSource(follower, Alliance.RED));
        final double launchVelocityBoost = 90;
        tasks = new Tasks(follower, intakeSubsystem, transferSubsystem, launcherSubsystem, bindingManager,
                () -> autoAim.getPollenBaseVelocity() + launchVelocityBoost,
                () -> autoAim.getNectarBaseVelocity() + launchVelocityBoost);

        registerSubsystem(autoAim);
        registerSubsystem(intakeSubsystem);
        registerSubsystem(transferSubsystem);
        registerSubsystem(launcherSubsystem);

        hive = new HiveTipDetector(hardwareMap, RED_AUDIENCE_CELL_TAGS, hiveTipThresholdDeg);

        PoseFactory red = PoseFactory.degrees();
        Pose shoot = Paths.POSE_SHOOT_RED_GARDEN;
        Pose sweepStart = red.of(sweepStartX, sweepY, 180);
        Pose sweepEnd = red.of(sweepEndX, sweepY, 180);

        taskMaster.addTask(
            new SequentialTask(
                tasks.autoGetLaunchAllTask(),                                       // 1. shoot the 4 preloads
                new FollowPathTask(follower, line(shoot, sweepStart).linear(shoot, sweepStart)),   // 2. turn to 180, go to the wall
                new DeferredTask(() -> extendIntake ? tasks.autoGetExtendIntakeTask() : new NullTask()),
                tasks.autoGetStartIntakeTask(),                                     // 3. intake on
                new FollowPathTask(follower, line(sweepStart, sweepEnd).linear(sweepStart, sweepEnd), sweepSpeed),
                new SleepTask(intakeDwellMs),                                       // 4. let the last ball come in
                tasks.autoGetStopIntakeTask(),
                tasks.autoGetRetractIntakeTask(),
                new DeferredTask(() -> shootPickedUp                                // 5. check the HIVE, shoot?, park
                    ? new SequentialTask(
                        new FollowPathTask(follower, line(sweepEnd, shoot).linear(sweepEnd, shoot)),
                        new RaceTask(                                               // look until we can tell, max hiveCheckMs
                            new WaitUntilTask(() -> hive.getState() != HiveTipDetector.State.UNKNOWN),
                            new SleepTask(hiveCheckMs)),
                        new DeferredTask(() -> {
                            HiveTipDetector.State st = hive.getState();
                            boolean shoot2 = st == HiveTipDetector.State.UP
                                    || (st == HiveTipDetector.State.UNKNOWN && shootIfHiveUnknown);
                            hiveDecision = st + (shoot2 ? " -> SHOOT" : " -> KEEP the balls");
                            return shoot2 ? tasks.autoGetLaunchAllTask() : new NullTask();
                        }),
                        new FollowPathTask(follower, line(shoot, Paths.POSE_PARK_RED_GARDEN)
                                .linear(shoot, Paths.POSE_PARK_RED_GARDEN)))
                    : new FollowPathTask(follower, line(sweepEnd, Paths.POSE_PARK_RED_GARDEN)
                                .linear(sweepEnd, Paths.POSE_PARK_RED_GARDEN)))
            )
        );
    }

    @Override
    public void init_loop() {
        super.init_loop();
        follower.update();
        hive.update();                 // records the baseline: Red audience CELL up, seen from the start/shoot spot
        hive.addTelemetry(telemetry);
    }

    @Override
    public void start() {
        hive.lockBaseline();
        follower.setPose(Paths.POSE_SHOOT_RED_GARDEN);
        super.start();
    }

    @Override
    public void stop() {
        super.stop();
        hive.close();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        telemetry.addData("Pose", "%.1f, %.1f, %.0f deg",
                follower.pose().x(), follower.pose().y(), Math.toDegrees(follower.pose().heading()));
        hive.update();
        hive.addTelemetry(telemetry);
        telemetry.addData("HIVE decision", hiveDecision);
        PoseStorage.saveFromAutonomous(follower.pose(), Alliance.RED);   // TeleOp picks this up at INIT
    }
}
