package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.HiveVisionSubsystem;

@TeleOp(name = "Hive State Test - Red Audience", group = "Testing")
public class HiveStateRedAudienceTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        HiveVisionSubsystem hive = new HiveVisionSubsystem(
                hardwareMap, HiveVisionSubsystem.Alliance.RED, HiveVisionSubsystem.RobotSide.AUDIENCE);

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
