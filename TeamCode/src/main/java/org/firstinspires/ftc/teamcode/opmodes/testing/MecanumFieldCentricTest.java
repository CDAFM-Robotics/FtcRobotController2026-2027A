package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrainSubsystem;

@Disabled   // Nov 20: hidden for competition; remove this line to use it
@TeleOp(name = "Mecanum Field Centric", group = "Testing")
public class MecanumFieldCentricTest extends ModularOpMode {

    private LocalizerSubsystem localizerSubsystem;
    private MecanumDriveTrainSubsystem mecanumDriveTrainSubsystem;

    @Override
    protected void preload() {
        localizerSubsystem = new LocalizerSubsystem(hardwareMap, "pinpoint");
        localizerSubsystem.setBindings(bindingManager);

        mecanumDriveTrainSubsystem = new MecanumDriveTrainSubsystem(hardwareMap, "frontLeftDriveMotor", "backLeftDriveMotor", "frontRightDriveMotor", "backRightDriveMotor");
        mecanumDriveTrainSubsystem.setBindings(bindingManager, () -> -gamepad1.left_stick_y, () -> gamepad1.left_stick_x, () -> gamepad1.right_stick_x, () -> 1, () -> bindingManager.checkAnalog("heading"));

        registerSubsystem(localizerSubsystem);
        registerSubsystem(mecanumDriveTrainSubsystem);
    }

    @Override
    public void loop() {
        super.loop();

        // Pose from the Pinpoint, set up from Pedro's Constants.localizerConfig. Push forward: x up. Push left: y up.
        // Turn left: heading up. Spin in place a full turn: x and y return close to where they started.
        telemetry.addData("x (in)", "%.2f", localizerSubsystem.getX());
        telemetry.addData("y (in)", "%.2f", localizerSubsystem.getY());
        telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(localizerSubsystem.getHeading()));
        telemetry.update();
    }
}
