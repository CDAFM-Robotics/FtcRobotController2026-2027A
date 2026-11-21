package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.HiveVisionSubsystemBasic;

@Disabled   // Nov 20: hidden for competition; remove this line to use it
@TeleOp(name = "Hive State Basic Test - Red NonAudience - Camera Only", group = "Testing")
public class HiveStateRedNonAudienceBasicTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        HiveVisionSubsystemBasic hive = new HiveVisionSubsystemBasic(
                hardwareMap, HiveVisionSubsystemBasic.Alliance.RED, HiveVisionSubsystemBasic.RobotSide.NON_AUDIENCE);

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
