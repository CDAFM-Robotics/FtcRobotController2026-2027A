package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.cdarobotics.cdalib.tasks.FollowPathTask;
import com.cdarobotics.cdalib.tasks.ParallelTask;
import com.cdarobotics.cdalib.tasks.SequentialTask;
import com.cdarobotics.cdalib.tasks.SleepTask;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.Paths;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.tasks.Tasks;

public class BlueGardenAutonomous extends ModularOpMode {

    private Follower follower;

    private IntakeSubsystem intakeSubsystem;
    private TransferSubsystem transferSubsystem;
    private LauncherSubsystem launcherSubsystem;

    private Tasks tasks;

    @Override
    protected void preload() {

        follower = Constants.createFollower(hardwareMap);

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, "intakeMotor", "leftExtenderServo", "rightExtenderServo");
        transferSubsystem = new TransferSubsystem(hardwareMap, "transferMotor", "leftTransferServo", "rightTransferServo");
        launcherSubsystem = new LauncherSubsystem(hardwareMap, telemetry, "pollenLauncherMotor", "nectarLauncherMotor");

        tasks = new Tasks(follower, intakeSubsystem, transferSubsystem, launcherSubsystem, bindingManager);

        taskMaster.addTask(
            new SequentialTask(
                new FollowPathTask(follower, Paths.BlueGardenPaths.startToShoot()),
                tasks.autoGetLaunchAllTask(),
                new ParallelTask(
                    tasks.autoGetStartIntakeTask(),
                    tasks.autoGetExtendIntakeTask(),
                    new FollowPathTask(follower, Paths.BlueGardenPaths.pickupGardenBlue())
                ),
                new SleepTask(500),
                tasks.autoGetRetractIntakeTask(),
                tasks.autoGetStopIntakeTask(),
                new FollowPathTask(follower, Paths.BlueGardenPaths.gardenToShoot()),
                tasks.autoGetLaunchAllTask(),
                new ParallelTask(
                    tasks.autoGetStartIntakeTask(),
                    tasks.autoGetExtendIntakeTask(),
                    new FollowPathTask(follower, Paths.BlueGardenPaths.pickupFlowerGarden())
                ),
                new SleepTask(500),
                tasks.autoGetRetractIntakeTask(),
                tasks.autoGetStopIntakeTask(),
                new FollowPathTask(follower, Paths.BlueGardenPaths.flowerGardenToShoot()),
                tasks.autoGetLaunchAllTask(),
                new FollowPathTask(follower, Paths.BlueGardenPaths.shootToPark())
            )
        );
    }
}
