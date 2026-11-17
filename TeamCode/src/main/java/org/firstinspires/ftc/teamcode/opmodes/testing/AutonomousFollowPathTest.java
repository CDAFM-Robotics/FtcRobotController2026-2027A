package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static com.pedropathing.api.Paths.line;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

@Autonomous(name = "Autonomous Follow Path Test", group = "Testing")
public class AutonomousFollowPathTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.createFollower(hardwareMap);

        waitForStart();

        follower.setPose(new Pose(0, 0, 0));

        follower.follow(line(new Pose(0, 0, 0), new Pose(10, 10, 0)).constant(0));

        while (opModeIsActive()) {
            follower.update();
        }
    }
}
