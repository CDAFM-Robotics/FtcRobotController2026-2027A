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

public class RedGardenAutonomous extends ModularOpMode {

    private Follower follower;

    @Override
    protected void preload() {

        follower = Constants.createFollower(hardwareMap);

        taskMaster.addTask(
            new SequentialTask(
                new FollowPathTask(follower, Paths.RedGardenPaths.startToShoot()),
                Tasks.autoGetLaunchAllTask(),
                new ParallelTask(
                    Tasks.autoGetStartIntakeTask(),
                    Tasks.autoGetExtendIntakeTask(),
                    new FollowPathTask(follower, Paths.RedGardenPaths.pickupGardenRed())
                ),
                new SleepTask(500),
                Tasks.autoGetRetractIntakeTask(),
                Tasks.autoGetStopIntakeTask(),
                new FollowPathTask(follower, Paths.RedGardenPaths.gardenToShoot()),
                Tasks.autoGetLaunchAllTask(),
                new ParallelTask(
                    Tasks.autoGetStartIntakeTask(),
                    Tasks.autoGetExtendIntakeTask(),
                    new FollowPathTask(follower, Paths.RedGardenPaths.pickupFlowerGarden())
                ),
                new SleepTask(500),
                Tasks.autoGetRetractIntakeTask(),
                Tasks.autoGetStopIntakeTask(),
                new FollowPathTask(follower, Paths.RedGardenPaths.flowerGardenToShoot()),
                Tasks.autoGetLaunchAllTask(),
                new FollowPathTask(follower, Paths.RedGardenPaths.shootToPark())
            )
        );
    }
}
