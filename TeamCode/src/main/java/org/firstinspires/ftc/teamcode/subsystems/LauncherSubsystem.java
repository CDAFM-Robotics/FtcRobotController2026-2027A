package org.firstinspires.ftc.teamcode.subsystems;

import com.cdarobotics.cdalib.bindings.BindingManager;
import com.cdarobotics.cdalib.devices.actuators.CRServoDevice;
import com.cdarobotics.cdalib.devices.actuators.MotorDevice;
import com.cdarobotics.cdalib.devices.actuators.ServoDevice;
import com.cdarobotics.cdalib.subsystems.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class LauncherSubsystem extends Subsystem {

    private final MotorDevice pollenLauncherMotor;
    private final MotorDevice nectarLauncherMotor;

    // private final CRServoDevice launcherCRServo;

    private BindingManager bindingManager;

    public LauncherSubsystem(HardwareMap hardwareMap, String pollenLauncherMotorName, String nectarLauncherMotorName, String launcherCRServoName) {
        pollenLauncherMotor = new MotorDevice(hardwareMap, pollenLauncherMotorName);
        nectarLauncherMotor = new MotorDevice(hardwareMap, nectarLauncherMotorName);

    }
    public void setBindings(BindingManager bindingManager, BooleanSupplier pollenMotor, BooleanSupplier nectarMotor, BooleanSupplier launcherTuningIncrease, BooleanSupplier launcherTuningDecrease, BooleanSupplier selectLauncherTuning) {
        bindingManager.addBinding("pollenMotor", pollenMotor);
        bindingManager.addBinding("nectarMotor", nectarMotor);
        bindingManager.addBinding("launcherTuningIncrease", launcherTuningIncrease);
        bindingManager.addBinding("launcherTuningDecrease", launcherTuningDecrease);
        bindingManager.addBinding("selectLauncherTuning", selectLauncherTuning);

        this.bindingManager = bindingManager;
    }

    @Override
    public void init() {
        // pollenLauncherMotor
        pollenLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pollenLauncherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        // NectarLauncherMotor
        nectarLauncherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        nectarLauncherMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // launcherCRServo = new CRServoDevice(hardwareMap, launcherCRServoName);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }
    boolean selectingTuning = false;
    boolean tuningPollenLauncher = true;
    boolean tuningChanging = false;
    double pollenLauncherPower = 0.5;
    double nectarLauncherPower = 0.5;
    
    double tuningAmount = 0.05;

    @Override
    public void update() {
        //code for fine-tuning the launcher power
        //could also integrate launcher angle or rotation tuning if applicable
        if (bindingManager.checkBinding("selectLauncherTuning") & !selectingTuning) {
            selectingTuning = true;
            tuningPollenLauncher = !tuningPollenLauncher;
        }
        else {
            selectingTuning = false;
        }
        if (bindingManager.checkBinding("launcherTuningIncrease")|bindingManager.checkBinding("launcherTuningDecrease") &!tuningChanging){
            tuningChanging = true;
            if (bindingManager.checkBinding("launcherTuningIncrease")&tuningPollenLauncher) {
                pollenLauncherPower += tuningAmount;
            }
            else if (bindingManager.checkBinding("launcherTuningIncrease")){
                nectarLauncherPower += tuningAmount;
            }
            else if (bindingManager.checkBinding("launcherTuningDecrease")&tuningPollenLauncher) {
                pollenLauncherPower -= tuningAmount;
            }
            else{
                nectarLauncherPower -= tuningAmount;
            }
        }
        else{
            tuningChanging = false;
        }


        if (bindingManager.checkBinding("pollenMotor")) {
            pollenLauncherMotor.setPower(pollenLauncherPower); //was 0.5
        }
        else {
            pollenLauncherMotor.setPower(0);
        }
        pollenLauncherMotor.update();

        if (bindingManager.checkBinding("nectarMotor")) {
            nectarLauncherMotor.setPower(nectarLauncherPower);//starts at same value as pollen launcher
        }
        else {
            nectarLauncherMotor.setPower(0);
        }
    }

    @Override
    public void stop() {


    }
}
