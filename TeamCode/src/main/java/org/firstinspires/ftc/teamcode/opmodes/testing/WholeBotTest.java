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

    // Per-shot feed gate: gamepad2 RT fires with the Pollen wheel, gamepad2 LT with both Pollen and Nectar. While either is
    // held, the transfer runs only when every wheel that trigger needs is within readyBandTicks below its target (overshoot
    // counts as ready). gp2 RB / LB pre-spin the wheels (Pollen / both) without feeding. After each ball the wheel dips 10-20 % and the
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

    // gp2 pre-spin: RB toggles Pollen only, LB toggles Pollen + Nectar. Spins the wheels without feeding, so the
    // first shot goes as soon as a fire trigger is pulled instead of waiting for spin-up from zero.
    private enum SpinMode { OFF, POLLEN, BOTH }
    private SpinMode spinMode = SpinMode.OFF;

    // gp2 rumble once each time the spinning wheels first reach speed (re-armed when the set of spinning wheels changes).
    public static int readyRumbleMs = 250;
    private int lastSpinSet = 0;          // 0 = off, 1 = Pollen only, 2 = Pollen + Nectar
    private boolean readyRumbled = false;

    @Override
    protected void preload() {
        launcherSubsystem = new LauncherSubsystem(
                hardwareMap, telemetry, "pollenLauncherMotor", "nectarLauncherMotor");
        launcherSubsystem.setBindings(bindingManager,
                this::pollenEnabled,
                this::nectarEnabled,
                () -> gamepad2.dpadUpWasPressed(),
                () -> gamepad2.dpadDownWasPressed(),
                () -> gamepad2.yWasPressed(),
                () -> gamepad2.aWasPressed());
        registerSubsystem(launcherSubsystem);

        transferSubsystem = new TransferSubsystem(
                hardwareMap, "transferMotor", "leftTransferServo", "rightTransferServo");
        transferSubsystem.setBindings(bindingManager,
                this::transferCommanded,     // servos: full power for launching and for nudging (unchanged)
                this::transferShouldRun);    // motor: transferMotorPower (1.0) only while launching; no dpad stepping
        // Motor while nudging: with the transferMotor binding off, TransferSubsystem runs the motor at autoNudgePower
        // (0.5) when a ball is detected or the gp1 RB nudge toggle is on (wheels off only), unless feedHold holds it.
        transferSubsystem.setNudgeRequest(() -> !anyWheelSpinning() && transferNudgeToggle);
        // The ball-detect auto-nudge inside TransferSubsystem is held (a) while any wheel spins and the fire gate is
        // closed (pre-spinning without a trigger, or firing with a wheel not back at speed), otherwise it would feed at
        // 0.5 power around the gate and fire a staged ball on its own; and (b) while gp1 reverses the intake, so the
        // transfer does not pull in a ball the intake is pushing out.
        transferSubsystem.setFeedHold(() -> (anyWheelSpinning() && !transferShouldRun()) || intakeReversing());
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

    /** gp1 LT: intake reverse to clear a jam. The only gp1 input that stops the transfer. */
    private boolean intakeReversing() {
        return gamepad1.left_trigger > 0.1;
    }

    /**
     * Transfer belt + servos at full feed power. Priority:
     *  1. gp1 intake reverse stops the transfer, always.
     *  2. A gp2 fire trigger with its wheels at speed feeds; gp1 intake forward / the Y macro does not block it.
     *  3. With every wheel stopped, a detected ball is staged, and gp1 RB's nudge toggle can run the transfer.
     *  While any wheel spins without an open fire gate, nothing feeds.
     */
    private boolean transferCommanded() {
        if (intakeReversing()) return false;
        if (transferShouldRun()) return true;
        return !anyWheelSpinning() && (transferNudgeToggle || transferSubsystem.hasGameElement());
    }

    /** gp2 RT: fire Pollen. */
    private boolean firePollen() {
        return gamepad2.right_trigger > 0.1;
    }

    /** gp2 LT: fire Pollen + Nectar. */
    private boolean fireBoth() {
        return gamepad2.left_trigger > 0.1;
    }

    private boolean isFiring() {
        return firePollen() || fireBoth();
    }

    /** Pollen wheel spins on either fire trigger or either pre-spin mode. */
    private boolean pollenEnabled() {
        return isFiring() || spinMode != SpinMode.OFF;
    }

    /** Nectar wheel spins on gp2 LT or the LB (both) pre-spin. */
    private boolean nectarEnabled() {
        return fireBoth() || spinMode == SpinMode.BOTH;
    }

    private boolean anyWheelSpinning() {
        return pollenEnabled() || nectarEnabled();
    }

    private boolean pollenReady() {
        return Math.abs(launcherSubsystem.getPollenCurrentVelocity()) >= launcherSubsystem.getPollenTargetVelocity() - readyBandTicks;
    }

    private boolean nectarReady() {
        return Math.abs(launcherSubsystem.getNectarCurrentVelocity()) >= launcherSubsystem.getNectarTargetVelocity() - readyBandTicks;
    }

    /** Every wheel the held fire trigger needs (RT: Pollen; LT: Pollen + Nectar) is within readyBandTicks below its
     *  target; above target counts as ready. A wheel that is only pre-spinning does not hold up an RT shot. */
    private boolean wheelReady() {
        return pollenReady() && (!fireBoth() || nectarReady());
    }

    /** Per-shot gate: feed only while a fire trigger is held and its wheels are ready. Pre-spin alone never feeds. No latch. */
    private boolean transferShouldRun() {
        return isFiring() && wheelReady();
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

        // gp2 rumble when the spinning wheels first reach speed.
        int spinSet = nectarEnabled() ? 2 : (pollenEnabled() ? 1 : 0);
        if (spinSet != lastSpinSet) { readyRumbled = false; lastSpinSet = spinSet; }
        boolean spinReady = (!pollenEnabled() || pollenReady()) && (!nectarEnabled() || nectarReady());
        if (spinSet != 0 && spinReady && !readyRumbled) {
            gamepad2.rumble(readyRumbleMs);
            readyRumbled = true;
        }

        // gp2 RB / LB: toggle pre-spin (Pollen only / Pollen + Nectar). Pressing the other bumper switches modes.
        if (gamepad2.rightBumperWasPressed()) spinMode = (spinMode == SpinMode.POLLEN) ? SpinMode.OFF : SpinMode.POLLEN;
        if (gamepad2.leftBumperWasPressed())  spinMode = (spinMode == SpinMode.BOTH)   ? SpinMode.OFF : SpinMode.BOTH;

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

        telemetry.addData("Macro state", macroState);
        telemetry.addData("Extender toggle", extenderToggle ? "EXTENDED" : "RETRACTED");
        telemetry.addData("Extender L feedback (V)", "%.3f", intakeSubsystem.getLeftFeedbackVoltage());
        telemetry.addData("Extender R feedback (V)", "%.3f", intakeSubsystem.getRightFeedbackVoltage());
        telemetry.addData("isExtended", intakeSubsystem.isExtended());
        telemetry.addData("isRetracted", intakeSubsystem.isRetracted());
        telemetry.addData("Triggers (gp2 RT / LT)", "%.2f / %.2f", gamepad2.right_trigger, gamepad2.left_trigger);
        telemetry.addData("Pollen flywheel", "%s  %.0f / %.0f", pollenEnabled() ? "ON" : "OFF",
                launcherSubsystem.getPollenCurrentVelocity(), launcherSubsystem.getPollenTargetVelocity());
        telemetry.addData("Nectar flywheel", "%s  %.0f / %.0f", nectarEnabled() ? "ON" : "OFF",
                launcherSubsystem.getNectarCurrentVelocity(), launcherSubsystem.getNectarTargetVelocity());
        telemetry.addData("Pre-spin (gp2 RB / LB)", spinMode);
        telemetry.addData("Transfer", transferShouldRun() ? "FEEDING" : (isFiring() ? "waiting for wheel" : "idle"));
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
        telemetry.addLine("gp2 RB: pre-spin pollen | gp2 LB: pre-spin both | gp2 RT: shoot pollen | gp2 LT: shoot pollen + nectar | dpad up/down: pollen vel +/-20 | Y/A: nectar vel +/-20");
        telemetry.addData("Transfer nudge", transferNudgeToggle ? "ON" : "off");
        telemetry.addLine("gp1 RT: intake fwd | gp1 LT: intake rev (stops transfer) | gp1 RB: transfer nudge (wheels off only) | gp1 LB: toggle extender | gp1 Y: intake macro");
        // One log line per gate transition, so shots can be lined up with the LauncherVelocity trace.
        boolean gateOpen = transferShouldRun();
        if (gateOpen != lastGateOpen) {
            android.util.Log.d("TransferGate", "t_ms=" + (System.nanoTime() / 1_000_000L) + " open=" + (gateOpen ? 1 : 0)
                    + " pollen_actual=" + (int) launcherSubsystem.getPollenCurrentVelocity()
                    + " pollen_target=" + (int) launcherSubsystem.getPollenTargetVelocity()
                    + " nectar_actual=" + (int) launcherSubsystem.getNectarCurrentVelocity()
                    + " nectar_target=" + (int) launcherSubsystem.getNectarTargetVelocity());
            lastGateOpen = gateOpen;
        }
        telemetry.addData("Feed gate", isFiring() ? (gateOpen ? "OPEN" : "waiting for wheel") : (anyWheelSpinning() ? "pre-spinning" : "launcher off"));
        LoopProfiler.mark("opmode");   // this OpMode's own controls and telemetry, above
        super.loop();
        LoopProfiler.mark("modules");  // ModularOpMode modules + taskMaster
        LoopProfiler.end();
    }
}
