package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.util.LoopProfiler;

@Configurable
@TeleOp(name = "Whole Bot Test", group = "Testing")
public class WholeBotTest extends ModularOpMode {

    private LauncherSubsystem launcherSubsystem;
    private TransferSubsystem transferSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private LocalizerSubsystem localizerSubsystem;
    private MecanumDriveTrainSubsystem mecanumDriveTrainSubsystem;

    // Per-shot feed gate: while gamepad2 RT is held, the transfer runs only when the Pollen wheel is within
    // readyBandTicks below target (overshoot counts as ready). After each ball the wheel dips 10-20 % and the
    // transfer pauses for the ~0.3 s it takes to recover, instead of pushing the next ball into a slow wheel.
    // Hub velocity comes in 20 ticks/s steps and the hold scatter is one step, so 40 = two steps.
    public static double readyBandTicks = 40;
    // Intake motor speed magnitude, live-tunable from Panels (from andre's 2c05819)
    public static double intakeMotorSpeed = 0.75;

    private enum MacroState { IDLE, EXTENDING, GRABBING, RETRACTING }
    private MacroState macroState = MacroState.IDLE;
    private final ElapsedTime macroTimer = new ElapsedTime();
    private static final double GRAB_TIMEOUT_SEC = 2.0;

    private boolean lastGateOpen        = false;
    private boolean transferNudgeToggle = false;
    private boolean extenderToggle      = false;
    private boolean prevRB = false;
    private boolean prevLB = false;
    private boolean prevY  = false;

    @Override
    protected void preload() {
        launcherSubsystem = new LauncherSubsystem(
                hardwareMap, telemetry, "pollenLauncherMotor", "nectarLauncherMotor");
        launcherSubsystem.setBindings(bindingManager,
                () -> gamepad2.right_trigger > 0.1,
                () -> gamepad2.right_trigger > 0.1,
                () -> gamepad2.dpadUpWasPressed(),
                () -> gamepad2.dpadDownWasPressed(),
                () -> gamepad2.yWasPressed(),
                () -> gamepad2.aWasPressed());
        registerSubsystem(launcherSubsystem);

        transferSubsystem = new TransferSubsystem(
                hardwareMap, "transferMotor", "leftTransferServo", "rightTransferServo");
        transferSubsystem.setBindings(bindingManager,
                () -> (transferShouldRun() || transferNudgeToggle || gamepad1.right_trigger > 0.1 || (!launcherEnabled() && transferSubsystem.hasGameElement())) && !isIntakeRunning(),
                () -> (transferShouldRun() || transferNudgeToggle || gamepad1.right_trigger > 0.1 || (!launcherEnabled() && transferSubsystem.hasGameElement())) && !isIntakeRunning(),
                () -> gamepad2.dpad_right,
                () -> gamepad2.dpad_left);
        // While shooting and the wheel is not back at speed, the ball-detect auto-nudge inside TransferSubsystem is
        // held too; otherwise it would feed at 0.5 power around the gate.
        transferSubsystem.setFeedHold(() -> launcherEnabled() && !wheelReady());
        registerSubsystem(transferSubsystem);

        intakeSubsystem = new IntakeSubsystem(
                hardwareMap, telemetry, "intakeMotor", "leftExtenderServo", "rightExtenderServo");
        intakeSubsystem.setBindings(bindingManager,
                () -> gamepad1.right_trigger > 0.1 ? intakeMotorSpeed
                    : (gamepad1.left_trigger > 0.1 ? -intakeMotorSpeed
                    : (macroState != MacroState.IDLE ? intakeMotorSpeed : 0.0)),
                () -> extenderToggle || macroState == MacroState.EXTENDING || macroState == MacroState.GRABBING);
        registerSubsystem(intakeSubsystem);

        localizerSubsystem = new LocalizerSubsystem(hardwareMap, "pinpoint");
        localizerSubsystem.setBindings(bindingManager);
        registerSubsystem(localizerSubsystem);

        mecanumDriveTrainSubsystem = new MecanumDriveTrainSubsystem(
                hardwareMap, "frontLeftDriveMotor", "backLeftDriveMotor", "frontRightDriveMotor", "backRightDriveMotor");
        mecanumDriveTrainSubsystem.setBindings(bindingManager,
                () -> -gamepad1.left_stick_y,
                () -> gamepad1.left_stick_x,
                () -> gamepad1.right_stick_x,
                () -> 1,
                localizerSubsystem::getHeading);
        registerSubsystem(mecanumDriveTrainSubsystem);
    }

    private boolean isIntakeRunning() {
        return gamepad1.right_trigger > 0.1 || gamepad1.left_trigger > 0.1 || macroState != MacroState.IDLE;
    }

    private boolean launcherEnabled() {
        return gamepad2.right_trigger > 0.1;
    }

    /** Pollen wheel at target, within readyBandTicks below it; above target counts as ready. */
    private boolean wheelReady() {
        double actual = Math.abs(launcherSubsystem.getPollenCurrentVelocity());
        double target = launcherSubsystem.getPollenTargetVelocity();
        return actual >= target - readyBandTicks;
    }

    /** Per-shot gate: feed only while the launcher is enabled and the wheel is ready. No latch. */
    private boolean transferShouldRun() {
        return launcherEnabled() && wheelReady();
    }

    @Override
    public void loop() {
        LoopProfiler.begin();
        // gp1 RB: toggle transfer nudge
        boolean rbNow = gamepad1.right_bumper;
        if (rbNow && !prevRB) transferNudgeToggle = !transferNudgeToggle;
        prevRB = rbNow;

        // gp1 LB: toggle extender
        boolean lbNow = gamepad1.left_bumper;
        if (lbNow && !prevLB) extenderToggle = !extenderToggle;
        prevLB = lbNow;

        // gp1 Y: start/cancel intake macro
        boolean yNow = gamepad1.y;
        if (yNow && !prevY) {
            if (macroState == MacroState.IDLE) {
                macroState = MacroState.EXTENDING;
                macroTimer.reset();
            } else {
                macroState = MacroState.IDLE;
            }
        }
        prevY = yNow;

        // Macro state machine
        switch (macroState) {
            case EXTENDING:
                if (intakeSubsystem.isExtended()) {
                    macroState = MacroState.GRABBING;
                    macroTimer.reset();
                }
                break;
            case GRABBING:
                if (macroTimer.seconds() >= GRAB_TIMEOUT_SEC) {
                    macroState = MacroState.RETRACTING;
                }
                break;
            case RETRACTING:
                if (intakeSubsystem.isRetracted()) {
                    macroState = MacroState.IDLE;
                }
                break;
            default:
                break;
        }

        double x = localizerSubsystem.getX();
        double y = localizerSubsystem.getY();

        // Telemetry
        double actualVelocity = launcherSubsystem.getPollenCurrentVelocity();
        double targetVelocity = launcherSubsystem.getPollenTargetVelocity();

        telemetry.addData("Macro state", macroState);
        telemetry.addData("Extender toggle", extenderToggle ? "EXTENDED" : "RETRACTED");
        telemetry.addData("Extender L feedback (V)", "%.3f", intakeSubsystem.getLeftFeedbackVoltage());
        telemetry.addData("Extender R feedback (V)", "%.3f", intakeSubsystem.getRightFeedbackVoltage());
        telemetry.addData("isExtended", intakeSubsystem.isExtended());
        telemetry.addData("isRetracted", intakeSubsystem.isRetracted());
        telemetry.addData("Trigger (gp2 RT)", "%.2f", gamepad2.right_trigger);
        telemetry.addData("Pollen flywheel", "%s  %.0f / %.0f",
                gamepad2.right_trigger > 0.1 ? "ON" : "OFF", actualVelocity, targetVelocity);
        telemetry.addData("Transfer", transferShouldRun() ? "FEEDING" : (launcherEnabled() ? "waiting for wheel" : "idle"));
        telemetry.addData("Transfer power", "%.1f", transferSubsystem.getTransferMotorPower());
        double distance = transferSubsystem.getDistance();
        if (!Double.isNaN(distance)) {
            telemetry.addData("Distance (cm)", "%.1f", distance);
            boolean elementDetected = transferSubsystem.hasGameElement();
            telemetry.addData("Game element", elementDetected ? "DETECTED" : "none");
            boolean autoNudgeActive = elementDetected && !transferNudgeToggle && !transferSubsystem.isFeedHeld();
            telemetry.addData("Auto nudge", autoNudgeActive ? "ACTIVE" : "off");
        } else {
            telemetry.addData("Distance sensor", "not found");
        }
        double distance2 = transferSubsystem.getDistance2();
        telemetry.addData("Distance2 (cm)", Double.isNaN(distance2) ? "not found" : String.format("%.1f", distance2));
        telemetry.addLine("gp2 RT: shoot | dpad up/down: pollen vel +/-20 | Y/A: nectar vel +/-20 | dpad left/right: transfer power -/+0.1");
        telemetry.addData("Transfer nudge", transferNudgeToggle ? "ON" : "off");
        telemetry.addLine("gp1 RT: intake fwd | gp1 LT: intake rev | gp1 RB: transfer nudge | gp1 LB: toggle extender | gp1 Y: intake macro");
        // One log line per gate transition, so shots can be lined up with the LauncherVelocity trace.
        boolean gateOpen = transferShouldRun();
        if (gateOpen != lastGateOpen) {
            android.util.Log.d("TransferGate", "t_ms=" + (System.nanoTime() / 1_000_000L) + " open=" + (gateOpen ? 1 : 0)
                    + " pollen_actual=" + (int) launcherSubsystem.getPollenCurrentVelocity()
                    + " pollen_target=" + (int) launcherSubsystem.getPollenTargetVelocity());
            lastGateOpen = gateOpen;
        }
        telemetry.addData("Feed gate", launcherEnabled() ? (gateOpen ? "OPEN" : "waiting for wheel") : "launcher off");
        LoopProfiler.mark("opmode");   // this OpMode's own controls and telemetry, above
        super.loop();
        LoopProfiler.mark("modules");  // ModularOpMode modules + taskMaster
        LoopProfiler.end();
    }
}
