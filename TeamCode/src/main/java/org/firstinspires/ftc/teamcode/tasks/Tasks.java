package org.firstinspires.ftc.teamcode.tasks;

import androidx.annotation.NonNull;

import com.cdarobotics.cdalib.tasks.NullTask;
import com.cdarobotics.cdalib.tasks.SequentialTask;
import com.cdarobotics.cdalib.tasks.Task;

public class Tasks {
    public static Task getLaunchPollenTask() {
        return new NullTask();
    }
    public static Task getLaunchNectarTask() {
        return new NullTask();
    }
    public static Task getLaunchAllTask() {
        return new NullTask();
    }

    public static Task autoGetLaunchPollenTask() {
        return new NullTask();
    }
    public static Task autoGetLaunchNectarTask() {
        return new NullTask();
    }
    public static Task autoGetLaunchAllTask() {
        return new NullTask();
    }

    public static Task autoGetStartIntakeTask() {
        return new NullTask();
    }
    public static Task autoGetStopIntakeTask() {
        return new NullTask();
    }
    public static Task autoGetReverseIntakeTask() {
        return new NullTask();
    }

    public static Task autoGetExtendIntakeTask() {
        return new NullTask();
    }
    public static Task autoGetRetractIntakeTask() {
        return new NullTask();
    }
}
