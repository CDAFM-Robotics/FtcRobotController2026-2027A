package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.HiveVisionSubsystem;

@Disabled   // Nov 20: hidden for competition; remove this line to use it
@TeleOp(name = "Hive State Test - Blue Non-Audience", group = "Testing")
public class HiveStateBlueNonAudienceTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        HiveVisionSubsystem hive = new HiveVisionSubsystem(
                hardwareMap, HiveVisionSubsystem.Alliance.BLUE, HiveVisionSubsystem.RobotSide.NON_AUDIENCE);

        telemetry.addLine("Ready - press START");
        telemetry.update();
        waitForStart();

        hive.initSample();

        while (opModeIsActive()) {
            hive.update();
            hive.addTelemetry(telemetry);
            telemetry.update();
        }

        hive.close();
    }
}
