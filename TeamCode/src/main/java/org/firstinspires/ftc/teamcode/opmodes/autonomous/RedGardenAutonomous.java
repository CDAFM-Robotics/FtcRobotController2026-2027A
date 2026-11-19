package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import static com.pedropathing.api.Paths.line;

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

// Red Garden (near) auto: shoot the preloaded balls, then park in the zone.
// No @Photon: PhotonCore swaps in motors without their configured type, reversing every goBILDA 5202 (drive included).
@Autonomous(name = "Red Garden", group = "0Competition")
public class RedGardenAutonomous extends ModularOpMode {

    private Follower follower;
    private IntakeSubsystem intakeSubsystem;
    private TransferSubsystem transferSubsystem;
    private LauncherSubsystem launcherSubsystem;
    private Tasks tasks;

    @Override
    protected void preload() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(Paths.POSE_SHOOT_RED_GARDEN);   // re-applied in start() after the Pinpoint IMU recalibration

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, "intakeMotor", "leftExtenderServo", "rightExtenderServo");
        transferSubsystem = new TransferSubsystem(hardwareMap, "transferMotor", "leftTransferServo", "rightTransferServo");
        launcherSubsystem = new LauncherSubsystem(hardwareMap, telemetry, "pollenLauncherMotor", "nectarLauncherMotor");

        // Auto-aim reads the Follower pose and feeds the launcher distance-based velocities; +90 over the table because
        // shots were landing short. Registered with no aim bindings, so it never auto-turns.
        AutoAimSubsystem autoAim = new AutoAimSubsystem(new FollowerPoseSource(follower, Alliance.RED));
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
                new FollowPathTask(follower, line(Paths.POSE_SHOOT_RED_GARDEN, Paths.POSE_PARK_RED_GARDEN)
                        .linear(Paths.POSE_SHOOT_RED_GARDEN, Paths.POSE_PARK_RED_GARDEN))
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
        follower.setPose(Paths.POSE_SHOOT_RED_GARDEN);
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
        PoseStorage.saveFromAutonomous(follower.pose(), Alliance.RED);   // TeleOp picks this up at INIT
    }
}
