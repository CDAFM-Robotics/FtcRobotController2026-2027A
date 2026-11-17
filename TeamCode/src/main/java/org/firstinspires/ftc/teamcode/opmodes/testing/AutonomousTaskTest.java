package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.cdarobotics.cdalib.tasks.FollowPathTask;
import com.cdarobotics.cdalib.tasks.SequentialTask;
import com.cdarobotics.cdalib.tasks.TaskMaster;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.pedropathing.Paths;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.tasks.Tasks;

@Photon
@Autonomous(name = "Autonomous Task Test", group = "Testing")
public class AutonomousTaskTest extends ModularOpMode {
    private Follower follower;

    private IntakeSubsystem intakeSubsystem;
    private TransferSubsystem transferSubsystem;
    private LauncherSubsystem launcherSubsystem;

    private Tasks tasks;

    private TelemetryManager panelsTelemetry;

    @Override
    protected void preload() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Pose is applied again in start(): createFollower() kicks off the Pinpoint's ~0.25s IMU
        // recalibration, which re-zeros the heading. Setting it here (during init) gets clobbered when
        // that recalibration finishes, so the follower would read heading 0 instead of the real start
        // heading and spin to "correct" it on the first path. This call just seeds init-screen telemetry.
        follower.setPose(Paths.POSE_START_RED_GARDEN);

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, "intakeMotor", "leftExtenderServo", "rightExtenderServo");
        transferSubsystem = new TransferSubsystem(hardwareMap, "transferMotor", "leftTransferServo", "rightTransferServo");
        launcherSubsystem = new LauncherSubsystem(hardwareMap, telemetry, "pollenLauncherMotor", "nectarLauncherMotor");

        tasks = new Tasks(follower, intakeSubsystem, transferSubsystem, launcherSubsystem, bindingManager);

        registerSubsystem(intakeSubsystem);
        registerSubsystem(transferSubsystem);
        registerSubsystem(launcherSubsystem);

        taskMaster.addTask(
            new SequentialTask(
                tasks.autoGetLaunchAllTask(1450),
                new FollowPathTask(follower, Paths.RedGardenPaths.pickupGardenRed())
            )
        );
    }

    @Override
    public void init_loop() {
        super.init_loop();
        follower.update();
        panelsTelemetry.addData("believed heading (deg)", Math.toDegrees(follower.pose().heading()));
        panelsTelemetry.addData("expected start heading (deg)", Math.toDegrees(Paths.POSE_START_RED_GARDEN.heading()));
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        // Re-apply the start pose now that PLAY is pressed. The Pinpoint's IMU recalibration (started
        // in createFollower() at init) has long finished, so this heading actually sticks on the chip.
        follower.setPose(Paths.POSE_START_RED_GARDEN);
        super.start();
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();

        double believed = Math.toDegrees(follower.pose().heading());
        panelsTelemetry.addData("x (in)", follower.pose().x());
        panelsTelemetry.addData("y (in)", follower.pose().y());
        panelsTelemetry.addData("believed heading (deg)", believed);
        if (follower.following()) {
            // closestPose() is the point on the path the follower is currently chasing, heading goal
            // included. If goal != believed while the robot spins, the path is commanding the turn;
            // if goal == believed but it spins anyway, the turn is coming from the drivetrain.
            double goal = Math.toDegrees(follower.closestPose().heading());
            panelsTelemetry.addData("goal heading (deg)", goal);
            panelsTelemetry.addData("heading error (deg)", believed - goal);
        }
        panelsTelemetry.update(telemetry);
    }
}
