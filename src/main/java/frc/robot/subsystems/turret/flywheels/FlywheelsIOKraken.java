package frc.robot.subsystems.turret.flywheels;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;

public class FlywheelsIOKraken implements FlywheelsIO {
    private Kraken frontWheelKraken;
    private Kraken frontWheelKrakenFollower;

    private double targetFrontWheelMPSLocal = 0.0;
    private boolean runningBangBangController = true;

    private VelocityTorqueCurrentFOC velTorqueFOC = new VelocityTorqueCurrentFOC(0.0).withSlot(0)
        .withUpdateFreqHz(60.0);

    private VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withSlot(1)
        .withEnableFOC(true).withUpdateFreqHz(60.0);

    public FlywheelsIOKraken() { 
        frontWheelKraken = new Kraken(TurretConstants.frontWheelKrakenID, UniversalConstants.canivoreName);
        frontWheelKrakenFollower = new Kraken(TurretConstants.frontWheelFollowerKrakenID, UniversalConstants.canivoreName);

        configFrontWheelKrakens();
    }

    private void configFrontWheelKrakens() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 130;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        //https://www.desmos.com/calculator/janq2cvkgo
        //old y=0.120913x+0.375351  new y=0.121122x+0.390959
        // volts
        config.Slot1.kS = 0.390959; // voltage to get over static friction
        config.Slot1.kV = 0.121122; // volts per rps
        config.Slot1.kP = 0.0;

        // current bang bang 
        // config.Slot2.kS = 0.369422; // voltage to get over static friction
        // config.Slot2.kV = 0.12203; // volts per rps
        // config.Slot2.kP = 0.55;

        config.Voltage.PeakForwardVoltage = 11.0;
        config.Voltage.PeakReverseVoltage = 0.0;


        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = TurretConstants.mainWheelKrakenToTurretRotationsGearRatio;
        frontWheelKraken.applyConfig(config);
        frontWheelKraken.getVelocity().setUpdateFrequency(1000);

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        frontWheelKrakenFollower.applyConfig(config);
        frontWheelKrakenFollower.getVelocity().setUpdateFrequency(250);
    }

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        inputs.frontWheelVelocityRPS = frontWheelKraken.getVelocity().getValueAsDouble();
        inputs.frontWheelFollowerRPS = frontWheelKrakenFollower.getVelocity().getValueAsDouble();

        inputs.frontWheelVelocityMPS = inputs.frontWheelVelocityRPS*(Math.PI*TurretConstants.mainFlywheelDiameterMeters);
        inputs.frontWheelFollowerVelocityMPS = inputs.frontWheelFollowerRPS*(Math.PI*TurretConstants.mainFlywheelDiameterMeters);

        inputs.targetFrontWheelVelocityMPS = targetFrontWheelMPSLocal;

        inputs.frontWheelAppliedVoltage = frontWheelKraken.getMotorVoltage().getValueAsDouble();
        inputs.frontWheelFollowerAppliedVoltage = frontWheelKrakenFollower.getMotorVoltage().getValueAsDouble();

        inputs.frontWheelAmps = frontWheelKraken.getStatorCurrent().getValueAsDouble();
        inputs.frontWheelFollowerAmps= frontWheelKrakenFollower.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setFrontWheelVolts(double volts) {
        frontWheelKraken.setVoltage(volts);
        frontWheelKrakenFollower.setVoltage(volts);
    }


    @Override
    public void setFrontWheelAmps(double amps) {
        frontWheelKraken.setControl(new TorqueCurrentFOC(amps).withUpdateFreqHz(0.0));
        frontWheelKrakenFollower.setControl(new TorqueCurrentFOC(amps));
    }

    @Override
    public void setTargetFrontWheelVelocity(double targetVelocityRPS) {
        targetFrontWheelMPSLocal = targetVelocityRPS*1.0 * (Math.PI*TurretConstants.mainFlywheelDiameterMeters);
        if(!(runningBangBangController)) {
            frontWheelKraken.setControl(velTorqueFOC.withVelocity(targetVelocityRPS*1.0));
            // even is you set the motors to clockwise and counterclockwise you still need to set opposed if opposed
            frontWheelKrakenFollower.setControl(new Follower(TurretConstants.frontWheelKrakenID, MotorAlignmentValue.Opposed));
        } else {
            frontWheelKraken.setControl(velocityVoltage.withVelocity(targetVelocityRPS));
            frontWheelKrakenFollower.setControl(velocityVoltage.withVelocity(targetVelocityRPS));
        }
    }
}
