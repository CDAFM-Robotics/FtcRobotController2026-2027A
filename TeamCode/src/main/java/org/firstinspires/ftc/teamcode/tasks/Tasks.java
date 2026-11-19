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

import java.util.function.DoubleSupplier;

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

    // How close each wheel must sit to its own target before we feed (ticks/s). Hub velocity arrives in ~20 ticks/s
    // steps and the flywheel PID settles with a little steady-state error, so we wait for "at speed", not an exact match.
    private static final double LAUNCHER_AT_SPEED_TOLERANCE = 30;

    /**
     * @param pollenBaseVelocity base Pollen target (ticks/s), e.g. AutoAimSubsystem::getPollenBaseVelocity. Return NaN
     *                           (no pose) to let the launcher use its fixed default. Pass {@code () -> Double.NaN} to
     *                           always use defaults.
     * @param nectarBaseVelocity base Nectar target (ticks/s); same NaN = default convention.
     */
    public Tasks(Follower follower, IntakeSubsystem intake, TransferSubsystem transfer, LauncherSubsystem launcher,
                 BindingManager bindingManager, DoubleSupplier pollenBaseVelocity, DoubleSupplier nectarBaseVelocity) {
        this.follower = follower;
        this.intake = intake;
        this.transfer = transfer;
        this.launcher = launcher;

        intake.setBindings(bindingManager, () -> intakeSpeed, () -> intakeExtend);
        transfer.setBindings(bindingManager, () -> transferServoActive, () -> transferActive);
        launcher.setBindings(bindingManager, () -> launcherActive, () -> launcherActive);
        // The launcher recomputes its target as base + manual offset every loop, so the automatic distance-based speed
        // is applied continuously while the wheels are on; the launch task just opens the gate and waits for "at speed".
        launcher.setVelocitySources(pollenBaseVelocity, nectarBaseVelocity);
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
        // Just open the gate: the launcher pulls its target from the velocity sources (AutoAim's distance table, or the
        // fixed defaults when a source returns NaN) every loop, so there is no fixed speed to set here.
        return new InstantTask(
            () -> launcherActive = true
        );
    }

    public Task autoStopLauncherTask() {
        return new InstantTask(
            () -> launcherActive = false
        );
    }

    /** Wait until each wheel is within tolerance of its own live target; Pollen and Nectar run at different speeds. */
    public Task autoWaitForLauncherVelocityTask() {
        return new WaitUntilTask(
            () -> launcher.getPollenCurrentVelocity() >= launcher.getPollenTargetVelocity() - LAUNCHER_AT_SPEED_TOLERANCE
               && launcher.getNectarCurrentVelocity() >= launcher.getNectarTargetVelocity() - LAUNCHER_AT_SPEED_TOLERANCE
        );
    }

    public Task autoGetLaunchAllTask() {
        return new SequentialTask(
            autoSpinUpLauncherTask(),
            autoWaitForLauncherVelocityTask(),
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
