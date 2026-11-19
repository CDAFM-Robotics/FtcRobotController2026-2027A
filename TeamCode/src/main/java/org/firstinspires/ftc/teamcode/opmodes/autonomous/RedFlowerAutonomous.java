package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static com.pedropathing.api.Paths.line;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.cdarobotics.cdalib.tasks.FollowPathTask;
import com.cdarobotics.cdalib.tasks.SequentialTask;
import com.cdarobotics.cdalib.tasks.SleepTask;
import com.pedropathing.follower.Follower;
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

// Red Flower (far) auto: wait for a tip, shoot the preload, then park in the zone.
// No @Photon: PhotonCore swaps in motors without their configured type, reversing every goBILDA 5202 (drive included).
@Autonomous(name = "Red Flower", group = "0Competition")
public class RedFlowerAutonomous extends ModularOpMode {

    private Follower follower;
    private IntakeSubsystem intakeSubsystem;
    private TransferSubsystem transferSubsystem;
    private LauncherSubsystem launcherSubsystem;
    private Tasks tasks;

    @Override
    protected void preload() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(Paths.POSE_SHOOT_RED_FLOWER);   // re-applied in start() after the Pinpoint IMU recalibration

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, "intakeMotor", "leftExtenderServo", "rightExtenderServo");
        transferSubsystem = new TransferSubsystem(hardwareMap, "transferMotor", "leftTransferServo", "rightTransferServo");
        launcherSubsystem = new LauncherSubsystem(hardwareMap, telemetry, "pollenLauncherMotor", "nectarLauncherMotor");

        // Auto-aim reads the Follower pose and feeds the launcher distance-based velocities; +40 over the table (the
        // Flower-side shot ran 50 hotter than it needed). Registered with no aim bindings, so it never auto-turns.
        AutoAimSubsystem autoAim = new AutoAimSubsystem(new FollowerPoseSource(follower, Alliance.RED));
        final double launchVelocityBoost = 40;
        tasks = new Tasks(follower, intakeSubsystem, transferSubsystem, launcherSubsystem, bindingManager,
                () -> autoAim.getPollenBaseVelocity() + launchVelocityBoost,
                () -> autoAim.getNectarBaseVelocity() + launchVelocityBoost);

        registerSubsystem(autoAim);
        registerSubsystem(intakeSubsystem);
        registerSubsystem(transferSubsystem);
        registerSubsystem(launcherSubsystem);

        taskMaster.addTask(
            new SequentialTask(
                new SleepTask(12000),                           // wait ~12 s for a tip
                tasks.autoGetLaunchAllTask(),                   // shoot the four preloaded balls
                new FollowPathTask(follower, line(Paths.POSE_SHOOT_RED_FLOWER, Paths.POSE_PARK_RED_FLOWER)
                        .linear(Paths.POSE_SHOOT_RED_FLOWER, Paths.POSE_PARK_RED_FLOWER))
            )
        );
    }

    @Override
    public void init_loop() {
        super.init_loop();
        follower.update();
    }

    @Override
    public void start() {
        follower.setPose(Paths.POSE_SHOOT_RED_FLOWER);
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        PoseStorage.saveFromAutonomous(follower.pose(), Alliance.RED);   // TeleOp picks this up at INIT
    }
}
