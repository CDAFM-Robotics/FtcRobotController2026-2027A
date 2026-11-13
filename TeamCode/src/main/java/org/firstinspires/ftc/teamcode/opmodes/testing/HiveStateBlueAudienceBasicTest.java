package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.HiveVisionSubsystem;
import org.firstinspires.ftc.teamcode.vision.HiveVisionSubsystemBasic;

@TeleOp(name = "Hive State Basic Test - Blue Audience - Camera Only", group = "Testing")
public class HiveStateBlueAudienceBasicTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        HiveVisionSubsystemBasic hive = new HiveVisionSubsystemBasic(
                hardwareMap, HiveVisionSubsystemBasic.Alliance.BLUE, HiveVisionSubsystemBasic.RobotSide.AUDIENCE);

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
