package org.firstinspires.ftc.teamcode.tasks;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.tasks.InstantTask;
import com.cdarobotics.cdalib.tasks.NullTask;
import com.cdarobotics.cdalib.tasks.Task;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class Tasks {

    private Follower follower;
    private IntakeSubsystem intake;
    private TransferSubsystem transfer;
    private LauncherSubsystem launcher;

    private double intakeSpeed;
    private boolean intakeExtend;

    private boolean transferActive;

    private boolean launcherActive;

    public Tasks(Follower follower, IntakeSubsystem intake, TransferSubsystem transfer, LauncherSubsystem launcher, BindingManager bindingManager) {
        this.follower = follower;
        this.intake = intake;
        this.transfer = transfer;
        this.launcher = launcher;

        intake.setBindings(bindingManager, () -> intakeSpeed, () -> intakeExtend);
        transfer.setBindings(bindingManager, () -> transferActive, () -> transferActive);
        launcher.setBindings(bindingManager, () -> launcherActive, () -> launcherActive);
    }




    public Task getLaunchPollenTask() {
        return new NullTask();
    }
    public Task getLaunchNectarTask() {
        return new NullTask();
    }
    public Task getLaunchAllTask() {
        return new NullTask();
    }

    public Task autoGetLaunchPollenTask() {
        return new NullTask();
    }
    public Task autoGetLaunchNectarTask() {
        return new NullTask();
    }

    public Task autoGetLaunchAllTask() {
        return new NullTask();
    }

    public Task autoGetStartIntakeTask() {
        return new InstantTask(
            () -> intakeSpeed = 1
        );
    }
    public Task autoGetStopIntakeTask() {
        return new InstantTask(
            () -> intakeSpeed = 0
        );
    }
    public Task autoGetReverseIntakeTask() {
        return new InstantTask(
            () -> intakeSpeed = -1
        );
    }

    public Task autoGetExtendIntakeTask() {
        return new InstantTask(
            () -> intakeExtend = true
        );
    }
    public Task autoGetRetractIntakeTask() {
        return new InstantTask(
            () -> intakeExtend = false
        );
    }
}
