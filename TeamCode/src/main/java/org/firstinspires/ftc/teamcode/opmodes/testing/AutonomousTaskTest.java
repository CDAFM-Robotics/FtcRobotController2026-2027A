package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.cdarobotics.cdalib.tasks.TaskMaster;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
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

    @Override
    protected void preload() {

        follower = Constants.createFollower(hardwareMap);

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, "intakeMotor", "leftExtenderServo", "rightExtenderServo");
        transferSubsystem = new TransferSubsystem(hardwareMap, "transferMotor", "leftTransferServo", "rightTransferServo");
        launcherSubsystem = new LauncherSubsystem(hardwareMap, telemetry, "pollenLauncherMotor", "nectarLauncherMotor");

        registerSubsystem(intakeSubsystem);
        registerSubsystem(transferSubsystem);
        registerSubsystem(launcherSubsystem);

        tasks = new Tasks(follower, intakeSubsystem, transferSubsystem, launcherSubsystem, bindingManager);

        taskMaster.addTask(tasks.autoStartTransferTask());
    }

    @Override
    public void loop() {
        super.loop();
        follower.update();
    }
}
