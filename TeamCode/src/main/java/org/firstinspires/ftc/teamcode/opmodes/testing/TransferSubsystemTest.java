package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

@TeleOp(name = "Transfer Subsystem Test", group = "Testing")
public class TransferSubsystemTest extends ModularOpMode {

    private TransferSubsystem transferSubsystem;


    @Override
    protected void preload() {
        transferSubsystem = new TransferSubsystem(hardwareMap, "transferMotor", "leftTransferServo", "rightTransferServo");

        transferSubsystem.setBindings(bindingManager, () -> gamepad1.right_bumper, () -> gamepad1.left_bumper);

        registerSubsystem(transferSubsystem);
    }

    @Override
    public void loop() {
        double distance = transferSubsystem.getDistance();
        if (!Double.isNaN(distance)) {
            telemetry.addData("Distance (cm)", "%.1f", distance);
            telemetry.addData("Game element", transferSubsystem.hasGameElement() ? "DETECTED" : "none");
        } else {
            telemetry.addData("Distance sensor", "not found");
        }
        telemetry.addLine("RB: servos | LB: motor");
        super.loop();
    }
}
