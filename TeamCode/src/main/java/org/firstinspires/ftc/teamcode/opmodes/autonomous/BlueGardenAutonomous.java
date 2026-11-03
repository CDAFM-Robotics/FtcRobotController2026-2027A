package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.cdarobotics.cdalib.tasks.FollowPathTask;
import com.cdarobotics.cdalib.tasks.ParallelTask;
import com.cdarobotics.cdalib.tasks.SequentialTask;
import com.cdarobotics.cdalib.tasks.SleepTask;
import com.cdarobotics.cdalib.tasks.Task;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.Paths;
import org.firstinspires.ftc.teamcode.tasks.Tasks;

public class BlueGardenAutonomous extends ModularOpMode {
    private Follower follower;

    @Override
    protected void preload() {

        follower = Constants.createFollower(hardwareMap);

        taskMaster.addTask(
            new SequentialTask(
                new FollowPathTask(follower, Paths.BlueGardenPaths.startToShoot()),
                Tasks.autoGetLaunchAllTask(),
                new ParallelTask(
                    Tasks.autoGetStartIntakeTask(),
                    Tasks.autoGetExtendIntakeTask(),
                    new FollowPathTask(follower, Paths.BlueGardenPaths.pickupGardenBlue())
                ),
                new SleepTask(500),
                Tasks.autoGetRetractIntakeTask(),
                Tasks.autoGetStopIntakeTask(),
                new FollowPathTask(follower, Paths.BlueGardenPaths.gardenToShoot()),
                Tasks.autoGetLaunchAllTask(),
                new ParallelTask(
                    Tasks.autoGetStartIntakeTask(),
                    Tasks.autoGetExtendIntakeTask(),
                    new FollowPathTask(follower, Paths.BlueGardenPaths.pickupFlowerGarden())
                ),
                new SleepTask(500),
                Tasks.autoGetRetractIntakeTask(),
                Tasks.autoGetStopIntakeTask(),
                new FollowPathTask(follower, Paths.BlueGardenPaths.flowerGardenToShoot()),
                Tasks.autoGetLaunchAllTask(),
                new FollowPathTask(follower, Paths.BlueGardenPaths.shootToPark())
            )
        );
    }
}
