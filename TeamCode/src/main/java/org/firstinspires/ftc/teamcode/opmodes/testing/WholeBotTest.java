package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

@Configurable
@TeleOp(name = "Whole Bot Test", group = "Testing")
public class WholeBotTest extends ModularOpMode {

    private LauncherSubsystem launcherSubsystem;
    private TransferSubsystem transferSubsystem;

    MecanumDriveTrainSubsystem mecanumDriveTrainSubsystem;

    private static final double RPM_READY_THRESHOLD = 0.9;
    private boolean transferLatched = false;
    private boolean transferNudgeToggle = false;
    private boolean extenderToggle = false;
    private boolean prevRB = false;
    private boolean prevLB = false;
    private double speedOfIntakeMotor;
    private boolean prevY = false;

    public static double intakeMotorSpeed = .75;

    @Override
    protected void preload() {
        launcherSubsystem = new LauncherSubsystem(
                hardwareMap, telemetry,"pollenLauncherMotor", "nectarLauncherMotor");
        launcherSubsystem.setBindings(bindingManager,
                () -> gamepad2.right_trigger > 0.1,
                () -> gamepad2.right_trigger > 0.1,
                () -> gamepad2.dpad_up,
                () -> gamepad2.dpad_down,
                () -> false,
                () -> false);
        registerSubsystem(launcherSubsystem);

        transferSubsystem = new TransferSubsystem(
                hardwareMap, "transferMotor", "leftTransferServo", "rightTransferServo");
        transferSubsystem.setBindings(bindingManager,
                () -> transferShouldRun() || transferNudgeToggle,
                () -> transferShouldRun() || transferNudgeToggle,
                () -> gamepad2.dpad_right,
                () -> gamepad2.dpad_left);
        registerSubsystem(transferSubsystem);

        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(
                hardwareMap, telemetry, "intakeMotor", "leftExtenderServo", "rightExtenderServo");
        intakeSubsystem.setBindings(bindingManager,
                () -> speedOfIntakeMotor,
                () -> extenderToggle);
        registerSubsystem(intakeSubsystem);

        mecanumDriveTrainSubsystem = new MecanumDriveTrainSubsystem(hardwareMap, "frontLeftDriveMotor", "backLeftDriveMotor", "frontRightDriveMotor", "backRightDriveMotor");

        mecanumDriveTrainSubsystem.setBindings(bindingManager, () -> -gamepad1.left_stick_y, () -> gamepad1.left_stick_x, () -> gamepad1.right_stick_x, () -> 1, () -> 0);

        registerSubsystem(mecanumDriveTrainSubsystem);
    }

    private boolean transferShouldRun() {
        if (gamepad2.right_trigger <= 0.1) {
            transferLatched = false;
            return false;
        }
        if (!transferLatched) {
            double actual = Math.abs(launcherSubsystem.getPollenCurrentVelocity());
            double target = launcherSubsystem.getPollenTargetVelocity();
            if (actual >= target * RPM_READY_THRESHOLD) transferLatched = true;
        }
        return transferLatched;
    }

    @Override
    public void loop() {
        boolean rbNow = gamepad1.right_bumper;
        if (rbNow && !prevRB) transferNudgeToggle = !transferNudgeToggle;
        prevRB = rbNow;

        //if we press lb it extends, when we dont it retracts
        boolean lbNow = gamepad1.left_bumper;
        if (lbNow) extenderToggle = true;
        else if (!lbNow) extenderToggle = false;
        prevLB = lbNow;

        double RTNow = gamepad1.right_trigger;
        double LTNow = gamepad1.left_trigger;
        //here were making it so if you press left trigger it goes backwards and if you press right trigger it goes forwards
        if(RTNow > 0.1 || LTNow > 0.1) speedOfIntakeMotor = RTNow > 0.1 ? intakeMotorSpeed : (LTNow > 0.1 ? -intakeMotorSpeed : 0.0);

        //if we press y it both extends the motor and turns on the intake
        boolean yNow = gamepad1.y;
        extenderToggle = yNow;
        speedOfIntakeMotor = yNow ? intakeMotorSpeed : 0;

        double actualVelocity = launcherSubsystem.getPollenCurrentVelocity();
        double targetVelocity = launcherSubsystem.getPollenTargetVelocity();
        telemetry.addData("Trigger (gp2 RT)", "%.2f", gamepad2.right_trigger);
        telemetry.addData("Pollen flywheel", "%s  %.0f / %.0f",
                gamepad2.right_trigger > 0.1 ? "ON" : "OFF", actualVelocity, targetVelocity);
        telemetry.addData("Transfer", transferLatched ? "FEEDING" : "waiting for flywheel");
        telemetry.addData("Transfer power", "%.1f", transferSubsystem.getTransferMotorPower());
        double distance = transferSubsystem.getDistance();
        if (!Double.isNaN(distance)) {
            telemetry.addData("Distance (cm)", "%.1f", distance);
            boolean elementDetected = transferSubsystem.hasGameElement();
            telemetry.addData("Game element", elementDetected ? "DETECTED" : "none");
            boolean autoNudgeActive = elementDetected && !transferNudgeToggle && !transferLatched;
            telemetry.addData("Auto nudge", autoNudgeActive ? "ACTIVE" : "off");
        } else {
            telemetry.addData("Distance sensor", "not found");
        }
        telemetry.addLine("gp2 RT: shoot | dpad up/down: velocity +/-50 | dpad left/right: transfer power -/+0.1");
        telemetry.addData("Transfer nudge", transferNudgeToggle ? "ON" : "off");
        telemetry.addData("Extender", extenderToggle ? "OUT" : "in");
        telemetry.addLine("gp1 RT: intake fwd | gp1 LT: intake rev | gp1 RB: toggle transfer nudge | gp1 LB: toggle extender");
        super.loop();
    }
}
