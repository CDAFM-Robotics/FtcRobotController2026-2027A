package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.cdarobotics.cdalib.opmodes.ModularOpMode;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

import java.util.function.DoubleSupplier;

@Configurable
@TeleOp(name = "Intake Test", group = "Testing")
//@Photon
public class IntakeSubsystemTest extends ModularOpMode {

    private IntakeSubsystem intakeSubsystem;

    private TransferSubsystem transferSubsystem;

    private TelemetryManager panelsTelemetry;

    int yellowBallsInIntake;
    int redBallsInIntake;
    int blueBallsInIntake;
    private double previousColorSensor1ColorGreen;
    private double previousColorSensor1ColorBlue;
    private double previousColorSensor1ColorRed;

    public double green;
    public double blue;
    public double red;

    public double differenceGreen;
    public double differenceBlue;
    public double differenceRed;

    @Override
    protected void preload() {
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry, "intakeMotor", "leftExtenderServo", "rightExtenderServo");

        intakeSubsystem.setBindings(bindingManager, () -> gamepad1.right_bumper, () -> gamepad1.left_bumper);

        registerSubsystem(intakeSubsystem);

        transferSubsystem = new TransferSubsystem(hardwareMap, "transferMotor", "leftTransferServo", "rightTransferServo");

        transferSubsystem.setBindings(bindingManager, () -> gamepad1.right_bumper || gamepad2.circle, () -> gamepad2.circle);

        registerSubsystem(transferSubsystem);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        super.loop();

        green = bindingManager.checkAnalog("colorSensorGreen");
        blue = bindingManager.checkAnalog("colorSensorBlue");
        red = bindingManager.checkAnalog("colorSensorRed");

        //difference calculations we will implement this at some point
        differenceGreen = green - previousColorSensor1ColorGreen;
        differenceBlue = blue - previousColorSensor1ColorBlue;
        differenceRed = red - previousColorSensor1ColorRed;

        //how many balls are there?
        if(yellowBallPickedUpRecently()){
            yellowBallsInIntake++;
        }
        if(blueBallPickedUpRecently()){
            blueBallsInIntake++;
        }
        if(redBallPickedUpRecently()){
            redBallsInIntake++;
        }

        //debugging so we can graph it on panels like bosses
        panelsTelemetry.debug("red:"+red);
        panelsTelemetry.debug("red difference:"+differenceRed);
        panelsTelemetry.debug("green:"+green);
        panelsTelemetry.debug("green difference:"+differenceGreen);
        panelsTelemetry.debug("blue:"+blue);
        panelsTelemetry.debug("blue difference:"+differenceBlue);

        panelsTelemetry.debug("red balls in intake:"+redBallsInIntake);
        panelsTelemetry.debug("blue balls in intake:"+blueBallsInIntake);
        panelsTelemetry.debug("yellow balls in intake:"+yellowBallsInIntake);

        telemetry.update();
        panelsTelemetry.update(telemetry);

        previousColorSensor1ColorGreen = bindingManager.checkAnalog("colorSensorGreen");
        previousColorSensor1ColorBlue = bindingManager.checkAnalog("colorSensorBlue");
        previousColorSensor1ColorRed = bindingManager.checkAnalog("colorSensorRed");
    }

    //functions in this test program will later be implemented in the transfer subsystem if theyre goated enough

    //TODO: Change constants for the color pickup for the new bot
    boolean redBallPickedUpRecently(){
        return (red >= .003 && differenceRed >= 0.002 && !yellowBallPickedUpRecently());
    }
    boolean yellowBallPickedUpRecently(){
        return(green >= .1 && differenceGreen >= .099);
    }
    boolean blueBallPickedUpRecently(){
        return(blue >= .003 && differenceGreen >= .002 && !yellowBallPickedUpRecently());
    }
}
