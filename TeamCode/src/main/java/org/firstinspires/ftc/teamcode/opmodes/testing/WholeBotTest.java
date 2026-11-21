package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.bylazar.configurables.annotations.Configurable;
import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AutoAimSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.util.LoopProfiler;

@Configurable
@Disabled   // Nov 20: hidden for competition; remove this line to use it
@TeleOp(name = "Whole Bot Test", group = "Testing")
public class WholeBotTest extends ModularOpMode {

    private LauncherSubsystem launcherSubsystem;
    private TransferSubsystem transferSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private LocalizerSubsystem localizerSubsystem;
    private AutoAimSubsystem autoAimSubsystem;
    private MecanumDriveTrainSubsystem mecanumDriveTrainSubsystem;

    // Per-shot feed gate: gamepad2 RT fires with the Pollen wheel, gamepad2 LT with both Pollen and Nectar. While either is
    // held, the transfer runs only when every wheel that trigger needs is within readyBandTicks below its target (overshoot
    // counts as ready). gp2 RB / LB pre-spin the wheels (Pollen / both) without feeding. After each ball the wheel dips 10-20 % and the
    // transfer pauses for the ~0.3 s it takes to recover, instead of pushing the next ball into a slow wheel.
    // Hub velocity comes in 20 ticks/s steps and the hold scatter is one step, so 40 = two steps.
    public static double readyBandTicks = 40;
    // Intake motor speed magnitude, live-tunable from Panels (from andre's 2c05819)
    public static double intakeMotorSpeed = 1.0;   // was 0.75; felt slow on Nov 14

    private enum MacroState { IDLE, EXTENDING, GRABBING, RETRACTING }
    private MacroState macroState = MacroState.IDLE;
    private final ElapsedTime macroTimer = new ElapsedTime();
    private static final double GRAB_TIMEOUT_SEC = 2.0;

    private boolean lastGateOpen        = false;
    private boolean extenderToggle      = false;
    private boolean prevRB = false;
    private boolean prevLB = false;
    private boolean prevY  = false;
    private boolean prevA  = false;

    // gp1 Y: Flower pickup toggle. On: extender retracts, Flower bar lowers, intake runs (all inside IntakeSubsystem).
    // Off: bar raises, intake stops 0.5 s later. Turning it on cancels the intake macro; A and LB are ignored while on.
    private boolean flowerMode = false;

    // gp1 rumble once per ball at transfer colorSensor1: fires when the sensor first sees a ball, re-arms after the
    // sensor has been clear for ballRumbleRearmSec (a passing ball can flicker the reading).
    public static int    ballRumbleMs       = 200;
    public static double ballRumbleRearmSec = 0.15;
    private boolean ballRumbleArmed = true;
    private final ElapsedTime sinceBallSeen = new ElapsedTime();

    // gp2 pre-spin: RB toggles Pollen only, LB toggles Pollen + Nectar. Spins the wheels without feeding, so the
    // first shot goes as soon as a fire trigger is pulled instead of waiting for spin-up from zero.
    private enum SpinMode { OFF, POLLEN, BOTH }
    private SpinMode spinMode = SpinMode.OFF;

    // gp2 rumble once each time the spinning wheels first reach speed (re-armed when the set of spinning wheels changes).
    public static int readyRumbleMs = 250;
    private int lastSpinSet = 0;          // 0 = off, 1 = Pollen only, 2 = Pollen + Nectar
    private boolean readyRumbled = false;

    // gp2 double rumble when aim (gp2 X held) reaches AIMED. Re-armed when X is released or the error grows past
    // aimRumbleRearmFactor x the aimed tolerance, so it doesn't buzz repeatedly at the edge of the tolerance.
    public static double aimRumbleRearmFactor = 2.0;
    private boolean aimRumbled = false;

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
        // when a ball is detected (wheels off only), unless feedHold holds it.
        // gp1 RB press: reverse ONE ball space - only the transfer motor runs backwards (servos off), measured by its
        // encoder (TransferSubsystem.reverseStepTicks). A launch feed cancels it.
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
        intakeSubsystem.setFlowerBinding(bindingManager, () -> flowerMode);
        registerSubsystem(intakeSubsystem);

        localizerSubsystem = new LocalizerSubsystem(hardwareMap, "pinpoint");
        // Autonomous handoff (PoseStorage at INIT), corner / heading resets and alliance choice. B and X are free on
        // gp1 here; they only pick the alliance before Start.
        localizerSubsystem.setBindings(bindingManager,
                () -> gamepad1.back,         // hold 0.5 s: Garden corner reset (Back / Share)
                () -> gamepad1.dpad_down,    // hold 0.5 s: heading reset, robot facing away from the drivers
                () -> gamepad1.b,            // before Start: Red
                () -> gamepad1.x,            // before Start: Blue
                gamepad1);
        registerSubsystem(localizerSubsystem);

        // Auto-aim at the CELL on our half of the field + launcher speed from distance. gp2 X held: robot turns so the
        // line between the launchers faces that CELL (which CELL is chosen from the pose, no button).
        // Registered before the drivetrain so the drive gets this loop's turn command.
        autoAimSubsystem = new AutoAimSubsystem(localizerSubsystem);
        autoAimSubsystem.setBindings(bindingManager,
                () -> gamepad2.x,
                () -> gamepad1.right_stick_x);
        launcherSubsystem.setVelocitySources(autoAimSubsystem::getPollenBaseVelocity, autoAimSubsystem::getNectarBaseVelocity);
        registerSubsystem(autoAimSubsystem);

        mecanumDriveTrainSubsystem = new MecanumDriveTrainSubsystem(
                hardwareMap, "frontLeftDriveMotor", "backLeftDriveMotor", "frontRightDriveMotor", "backRightDriveMotor");
        mecanumDriveTrainSubsystem.setBindings(bindingManager,
                () -> -gamepad1.left_stick_y,
                () -> gamepad1.left_stick_x,
                autoAimSubsystem::getTurn,          // driver's right stick, or auto-turn while gp2 X is held
                () -> 1,
                localizerSubsystem::getDriveHeading);   // alliance-adjusted; 0 (robot-centric) until alliance + heading known
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
     *  3. With every wheel stopped, a detected ball is staged. (gp1 RB's reverse nudge never runs the servos.)
     *  While any wheel spins without an open fire gate, nothing feeds.
     */
    private boolean transferCommanded() {
        if (intakeReversing()) return false;
        if (transferShouldRun()) return true;
        return !anyWheelSpinning() && !transferSubsystem.isReversing() && transferSubsystem.hasGameElement();
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
    public void init_loop() {
        super.init_loop();
        localizerSubsystem.addTelemetry(telemetry);
    }

    @Override
    public void loop() {
        LoopProfiler.begin();
        // gp1 RB: reverse the transfer one ball space (motor backwards only)
        boolean rbNow = gamepad1.right_bumper;
        if (rbNow && !prevRB) transferSubsystem.startReverseStep();
        prevRB = rbNow;

        // gp1 Y: toggle Flower pickup. Turning it on cancels the intake macro and the extender toggle.
        boolean yNow = gamepad1.y;
        if (yNow && !prevY) {
            flowerMode = !flowerMode;
            if (flowerMode) {
                macroState = MacroState.IDLE;
                extenderToggle = false;
            }
        }
        prevY = yNow;

        // gp1 LB: toggle extender (ignored during Flower pickup)
        boolean lbNow = gamepad1.left_bumper;
        if (lbNow && !prevLB && !flowerMode) extenderToggle = !extenderToggle;
        prevLB = lbNow;

        // gp1 A: start/cancel intake macro (ignored during Flower pickup)
        boolean aNow = gamepad1.a;
        if (aNow && !prevA && !flowerMode) {
            if (macroState == MacroState.IDLE) {
                macroState = MacroState.EXTENDING;
                macroTimer.reset();
            } else {
                macroState = MacroState.IDLE;
            }
        }
        prevA = aNow;

        // gp1 rumble once per ball at transfer colorSensor1.
        if (transferSubsystem.hasBallAtSensor1()) {
            if (ballRumbleArmed) {
                gamepad1.rumble(ballRumbleMs);
                ballRumbleArmed = false;
            }
            sinceBallSeen.reset();
        } else if (!ballRumbleArmed && sinceBallSeen.seconds() >= ballRumbleRearmSec) {
            ballRumbleArmed = true;
        }

        // gp2 rumble when the spinning wheels first reach speed.
        int spinSet = nectarEnabled() ? 2 : (pollenEnabled() ? 1 : 0);
        if (spinSet != lastSpinSet) { readyRumbled = false; lastSpinSet = spinSet; }
        boolean spinReady = (!pollenEnabled() || pollenReady()) && (!nectarEnabled() || nectarReady());
        if (spinSet != 0 && spinReady && !readyRumbled) {
            gamepad2.rumble(readyRumbleMs);
            readyRumbled = true;
        }

        // gp2 double rumble when aim locks on.
        if (autoAimSubsystem.isAiming() && autoAimSubsystem.isAimed()) {
            if (!aimRumbled) {
                gamepad2.rumbleBlips(2);
                aimRumbled = true;
            }
        } else if (!autoAimSubsystem.isAiming()
                || !(Math.abs(autoAimSubsystem.getHeadingErrorDeg()) <= AutoAimSubsystem.aimedToleranceDeg * aimRumbleRearmFactor)) {
            aimRumbled = false;
        }

        // gp2 RB / LB: pre-spin. With every wheel off, RB starts Pollen only and LB starts Pollen + Nectar. While any
        // wheel spins, either bumper turns pre-spin off, so one press always stops. (Nov 15: the old per-bumper toggle
        // let RB/LB alternate Pollen <-> Both without ever reaching off.) A held fire trigger still spins its wheels.
        boolean rbPressed = gamepad2.rightBumperWasPressed();
        boolean lbPressed = gamepad2.leftBumperWasPressed();
        if (rbPressed || lbPressed) {
            if (anyWheelSpinning()) spinMode = SpinMode.OFF;
            else spinMode = rbPressed ? SpinMode.POLLEN : SpinMode.BOTH;
        }

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

        // gp2 B: clear the +/-20 speed offsets back to 0 (target = table speed)
        if (gamepad2.bWasPressed()) launcherSubsystem.resetOffsets();

        // Telemetry

        localizerSubsystem.addTelemetry(telemetry);
        autoAimSubsystem.addTelemetry(telemetry);
        telemetry.addData("Speed offset Pollen / Nectar (gp2 B clears)", "%+.0f / %+.0f",
                launcherSubsystem.getPollenOffset(), launcherSubsystem.getNectarOffset());
        telemetry.addData("Flower pickup (gp1 Y)", !flowerMode ? "off"
                : (intakeSubsystem.isFlowerWaitingForRetract() ? "ON - retracting extender" : "ON - bar down, intaking"));
        telemetry.addData("Flower bar", !intakeSubsystem.isFlowerBarConfigured() ? "NOT CONFIGURED (flowerBarServo)"
                : (intakeSubsystem.isFlowerBarDown() ? "DOWN" : (intakeSubsystem.isFlowerBarUp() ? "UP" : "raising")));
        telemetry.addData("Ball at sensor 1", transferSubsystem.hasBallAtSensor1() ? "YES" : "no");
        telemetry.addData("Macro state", macroState);
        telemetry.addData("Extender toggle", extenderToggle ? "EXTENDED" : "RETRACTED");
        telemetry.addData("Extender L feedback (V)", "%.3f", intakeSubsystem.getLeftFeedbackVoltage());
        telemetry.addData("Extender R feedback (V)", "%.3f", intakeSubsystem.getRightFeedbackVoltage());
        telemetry.addData("isExtended", intakeSubsystem.isExtended());
        telemetry.addData("isRetracted", intakeSubsystem.isRetracted());
        telemetry.addData("Retract correction L / R", "%.3f / %.3f",
                intakeSubsystem.getRetractCorrectionLeft(), intakeSubsystem.getRetractCorrectionRight());
        if (intakeSubsystem.isRetractCorrectionAtLimit()) {
            telemetry.addLine("Extender: can't reach retracted (at INIT limit)");
        }
        telemetry.addData("Triggers (gp2 RT / LT)", "%.2f / %.2f", gamepad2.right_trigger, gamepad2.left_trigger);
        telemetry.addData("Pollen flywheel", "%s  %.0f / %.0f", pollenEnabled() ? "ON" : "OFF",
                launcherSubsystem.getPollenCurrentVelocity(), launcherSubsystem.getPollenTargetVelocity());
        telemetry.addData("Nectar flywheel", "%s  %.0f / %.0f", nectarEnabled() ? "ON" : "OFF",
                launcherSubsystem.getNectarCurrentVelocity(), launcherSubsystem.getNectarTargetVelocity());
        telemetry.addData("Pre-spin (gp2 RB / LB)", spinMode);
        telemetry.addData("Transfer mode", transferSubsystem.getMode());   // FULL (launch) / NUDGE / HELD / OFF
        telemetry.addData("Transfer motor / servos", "%.2f / %.2f",
                transferSubsystem.getMotorCommand(), transferSubsystem.getServoCommand());
        telemetry.addData("Launch gate", transferShouldRun() ? "FEEDING" : (isFiring() ? "waiting for wheel" : "idle"));
        double distance = transferSubsystem.getDistance();
        if (!Double.isNaN(distance)) {
            telemetry.addData("Distance (cm)", "%.1f", distance);
            boolean elementDetected = transferSubsystem.hasGameElement();
            telemetry.addData("Game element", elementDetected ? "DETECTED" : "none");
            boolean autoNudgeActive = elementDetected && !transferSubsystem.isReversing() && !transferSubsystem.isFeedHeld();
            telemetry.addData("Auto nudge", autoNudgeActive ? "ACTIVE" : "off");
        } else {
            telemetry.addData("Distance sensor", "not found");
        }
        double distance2 = transferSubsystem.getDistance2();
        telemetry.addData("Distance2 (cm)", Double.isNaN(distance2) ? "not found" : String.format("%.1f", distance2));
        telemetry.addLine("gp2 RB: pre-spin pollen | gp2 LB: pre-spin both | gp2 RT: shoot pollen | gp2 LT: shoot pollen + nectar | dpad up/down: pollen vel +/-20 | Y/A: nectar vel +/-20");
        telemetry.addData("Reverse step (gp1 RB)", transferSubsystem.isReversing() ? "RUNNING"
                : String.format("last %d / %d ticks%s", transferSubsystem.getLastReverseTicks(),
                        TransferSubsystem.reverseStepTicks, transferSubsystem.didLastReverseTimeOut() ? " (TIMED OUT)" : ""));
        telemetry.addData("Transfer encoder", transferSubsystem.getMotorPosition());
        telemetry.addLine("gp1 RT: intake fwd | gp1 LT: intake rev (stops transfer) | gp1 RB: reverse transfer 1 ball (motor only) | gp1 LB: toggle extender | gp1 A: intake macro | gp1 Y: Flower pickup on/off");
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
