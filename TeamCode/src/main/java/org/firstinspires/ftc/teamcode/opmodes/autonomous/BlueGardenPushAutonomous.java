package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static com.pedropathing.api.Paths.curve;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.cdarobotics.cdalib.tasks.FollowPathTask;
import com.cdarobotics.cdalib.tasks.SequentialTask;
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

// Blue Garden push auto, 180-deg rotation of Red Garden Push: shoot the preload from the Garden shoot pose, then the
// final trajectory drives up and shoves the adjacent robot into the loading zone instead of parking.
// No @Photon: PhotonCore swaps in motors without their configured type, reversing every goBILDA 5202 (drive included).
@Autonomous(name = "Blue Garden Push", group = "0Competition")
public class BlueGardenPushAutonomous extends ModularOpMode {

    private Follower follower;
    private IntakeSubsystem intakeSubsystem;
    private TransferSubsystem transferSubsystem;
    private LauncherSubsystem launcherSubsystem;
    private Tasks tasks;

    @Override
    protected void preload() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(Paths.POSE_SHOOT_BLUE_NEAR);   // re-applied in start() after the Pinpoint IMU recalibration

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, "intakeMotor", "leftExtenderServo", "rightExtenderServo");
        transferSubsystem = new TransferSubsystem(hardwareMap, "transferMotor", "leftTransferServo", "rightTransferServo");
        launcherSubsystem = new LauncherSubsystem(hardwareMap, telemetry, "pollenLauncherMotor", "nectarLauncherMotor");

        // Auto-aim reads the Follower pose and feeds the launcher distance-based velocities; +90 over the table because
        // shots were landing short. Registered with no aim bindings, so it never auto-turns.
        AutoAimSubsystem autoAim = new AutoAimSubsystem(new FollowerPoseSource(follower, Alliance.BLUE));
        final double launchVelocityBoost = 90;
        tasks = new Tasks(follower, intakeSubsystem, transferSubsystem, launcherSubsystem, bindingManager,
                () -> autoAim.getPollenBaseVelocity() + launchVelocityBoost,
                () -> autoAim.getNectarBaseVelocity() + launchVelocityBoost);

        registerSubsystem(autoAim);
        registerSubsystem(intakeSubsystem);
        registerSubsystem(transferSubsystem);
        registerSubsystem(launcherSubsystem);

        taskMaster.addTask(
            new SequentialTask(
                tasks.autoGetLaunchAllTask(),                    // shoot the four preloaded balls
                // Final trajectory (the only deviation from Blue Garden): curve to the loading-zone approach and shove
                // the adjacent robot into the loading zone, ending facing the wall.
                new FollowPathTask(follower, curve(Paths.POSE_SHOOT_BLUE_NEAR, Paths.POSE_PUSH_START_BLUE_GARDEN,
                        Paths.POSE_PUSH_END_BLUE_GARDEN).linear(Paths.POSE_SHOOT_BLUE_NEAR, Paths.POSE_PUSH_END_BLUE_GARDEN))
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
        follower.setPose(Paths.POSE_SHOOT_BLUE_NEAR);
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        PoseStorage.saveFromAutonomous(follower.pose(), Alliance.BLUE);   // TeleOp picks this up at INIT
    }
}
