package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LimelightDetectorSubsystem;

@TeleOp(name = "Limelight Pollen Detector Test", group = "Testing")
public class LimelightPollenDetectorTest extends ModularOpMode {

    private LimelightDetectorSubsystem limelightSubsystem;

    @Override
    protected void preload() {
        limelightSubsystem = new LimelightDetectorSubsystem(hardwareMap.get(Limelight3A.class, "Limelight"), telemetry);
        registerSubsystem(limelightSubsystem);



    }

    @Override
    public void loop() {
        super.loop();

        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
