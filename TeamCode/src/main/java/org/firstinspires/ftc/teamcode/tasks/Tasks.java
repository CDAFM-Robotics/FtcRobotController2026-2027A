package org.firstinspires.ftc.teamcode.tasks;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.tasks.InstantTask;
import com.cdarobotics.cdalib.tasks.NullTask;
import com.cdarobotics.cdalib.tasks.SequentialTask;
import com.cdarobotics.cdalib.tasks.SleepTask;
import com.cdarobotics.cdalib.tasks.Task;
import com.cdarobotics.cdalib.tasks.WaitUntilTask;
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

    private boolean transferServoActive;
    private boolean transferActive;

    private boolean launcherActive;

    public Tasks(Follower follower, IntakeSubsystem intake, TransferSubsystem transfer, LauncherSubsystem launcher, BindingManager bindingManager) {
        this.follower = follower;
        this.intake = intake;
        this.transfer = transfer;
        this.launcher = launcher;

        intake.setBindings(bindingManager, () -> intakeSpeed, () -> intakeExtend);
        transfer.setBindings(bindingManager, () -> transferServoActive, () -> transferActive);
        launcher.setBindings(bindingManager, () -> launcherActive, () -> launcherActive);
    }




    public Task autoStartTransferTask() {
        return new InstantTask(
            () -> transferActive = true
        );
    }

    public Task autoStopTransferTask() {
        return new InstantTask(
            () -> transferActive = false
        );
    }

    public Task autoStartTransferServoTask() {
        return new InstantTask(
            () -> transferServoActive = true
        );
    }

    public Task autoStopTransferServoTask() {
        return new InstantTask(
            () -> transferServoActive = false
        );
    }

    public Task autoSpinUpLauncherTask() {
        return new InstantTask(
            () -> launcherActive = true
        );
    }

    public Task autoStopLauncherTask() {
        return new InstantTask(
            () -> launcherActive = false
        );
    }

    public Task autoWaitForLauncherVelocityTask(double velocity) {
        return new WaitUntilTask(
            () -> launcher.getPollenCurrentVelocity() >= velocity && launcher.getNectarCurrentVelocity() >= velocity
        );
    }

    public Task autoGetLaunchAllTask(double velocity) {
        return new SequentialTask(
            autoSpinUpLauncherTask(),
            autoWaitForLauncherVelocityTask(velocity),
            autoStartTransferTask(),
            autoStartTransferServoTask(),
            new SleepTask(1000),
            autoStopLauncherTask(),
            autoStopTransferTask(),
            autoStopTransferServoTask()
        );
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
