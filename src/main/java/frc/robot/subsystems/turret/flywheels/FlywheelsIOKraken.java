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
    private Kraken hoodWheelKraken;

    private double targetFrontWheelMPSLocal = 0.0;
    private double targetHoodWheelMPSLocal = 0.0;

    // private double bangBangControllerVolts = 11.0;
    // private double bangBangControllerDeadzoneMPS = 0.1;
    private boolean runningBangBangController = true;

    private VelocityTorqueCurrentFOC velTorqueFOC = new VelocityTorqueCurrentFOC(0.0).withSlot(0)
        .withUpdateFreqHz(100.0);

    private VelocityVoltage velocityVoltage = new VelocityVoltage(0.0).withSlot(1)
        .withEnableFOC(true).withUpdateFreqHz(100.0);

    private VelocityTorqueCurrentFOC torqueCurrentBangBang = new VelocityTorqueCurrentFOC(0.0).withSlot(2)
        .withUpdateFreqHz(100.0);

    public FlywheelsIOKraken() { 
        frontWheelKraken = new Kraken(TurretConstants.frontWheelKrakenID, UniversalConstants.canivoreName);
        frontWheelKrakenFollower = new Kraken(TurretConstants.frontWheelFollowerKrakenID, UniversalConstants.canivoreName);
        hoodWheelKraken = new Kraken(TurretConstants.hoodWheelKrakenID, UniversalConstants.canivoreName);

        configFrontWheelKrakens();
        configHoodWheelKrakens();
    }

    private void configFrontWheelKrakens() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 200;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 4.0; // 4.0 amps to get over friction
        config.Slot0.kP = 4.7; // 8 amps per erreor of 1 rps
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.01;

        // for bang-bang controller in volts
        config.Slot1.kS = 0.268738; // voltage to get over static friction
        config.Slot1.kV = 0.119992; // volts per rps
        // config.Slot1.kP = 99999999.0;

        // current bang bang 
        config.Slot2.kS = 4.0;
        config.Slot2.kP = 99999999.0;

        if(runningBangBangController) {
            config.Voltage.PeakForwardVoltage = 11.0;
            config.Voltage.PeakReverseVoltage = 0.0;
        }

        config.TorqueCurrent.PeakForwardTorqueCurrent = 100;
        config.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
        config.TorqueCurrent.TorqueNeutralDeadband = 0.0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = TurretConstants.mainWheelKrakenToTurretRotationsGearRatio;
        frontWheelKraken.applyConfig(config);
        frontWheelKraken.getVelocity().setUpdateFrequency(250);

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        frontWheelKrakenFollower.applyConfig(config);
        frontWheelKrakenFollower.getVelocity().setUpdateFrequency(250);
    }

    private void configHoodWheelKrakens() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 200;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 3.6; // 5.0 amps to get over friction
        config.Slot0.kP = 11.3; // 8 amps per erreor of 1 rps
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.01;

        // for bang-bang controller in volts
        config.Slot1.kS = 0.292215; // voltage to get over static friction
        config.Slot1.kV = 0.121075; // volts per rps
        // config.Slot1.kP = 1.0;

        // current bang bang 
        config.Slot2.kS = 3.6;
        config.Slot2.kP = 99999999.0;

        if(runningBangBangController) {
            config.Voltage.PeakForwardVoltage = 11.0;
            config.Voltage.PeakReverseVoltage = 0.0;
            // config.Voltage.PeakForwardVoltage = 11.0;
            // config.Voltage.PeakReverseVoltage = -11.0;
            // config.Voltage.SupplyVoltageTimeConstant
        }

        config.TorqueCurrent.PeakForwardTorqueCurrent = 100;
        config.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
        config.TorqueCurrent.TorqueNeutralDeadband = 0.0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = TurretConstants.hoodWheelKrakenToTurretRotationsGearRatio;
        // config.ClosedLoopGeneral.ContinuousWrap = true;
        hoodWheelKraken.applyConfig(config);
        hoodWheelKraken.getVelocity().setUpdateFrequency(250);
    }

    @Override
    public void updateInputs(FlywheelsIOInputs inputs) {
        inputs.frontWheelVelocityRPS = frontWheelKraken.getVelocity().getValueAsDouble();
        inputs.frontWheelFollowerRPS = frontWheelKrakenFollower.getVelocity().getValueAsDouble();
        inputs.hoodWheelVelocityRPS = hoodWheelKraken.getVelocity().getValueAsDouble();

        inputs.frontWheelVelocityMPS = inputs.frontWheelVelocityRPS*(Math.PI*TurretConstants.mainFlywheelDiameterMeters);
        inputs.frontWheelFollowerVelocityMPS = inputs.frontWheelFollowerRPS*(Math.PI*TurretConstants.mainFlywheelDiameterMeters);
        inputs.hoodWheelVelocityMPS = inputs.hoodWheelVelocityRPS*(Math.PI*TurretConstants.hoodFlywheelDiameterMeters);

        inputs.targetFrontWheelVelocityMPS = targetFrontWheelMPSLocal;
        inputs.targetHoodWheelVelocityMPS = targetHoodWheelMPSLocal;

        inputs.frontWheelAppliedVoltage = frontWheelKraken.getMotorVoltage().getValueAsDouble();
        inputs.frontWheelFollowerAppliedVoltage = frontWheelKrakenFollower.getMotorVoltage().getValueAsDouble();
        inputs.hoodWheelAppledVoltage = hoodWheelKraken.getMotorVoltage().getValueAsDouble();

        inputs.frontWheelAmps = frontWheelKraken.getStatorCurrent().getValueAsDouble();
        inputs.frontWheelFollowerAmps= frontWheelKrakenFollower.getStatorCurrent().getValueAsDouble();
        inputs.hoodWheelAmps = hoodWheelKraken.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setFrontWheelVolts(double volts) {
        frontWheelKraken.setVoltage(volts);
        frontWheelKrakenFollower.setVoltage(volts);
    }

    @Override
    public void setHoodWheelVolts(double volts) {
        hoodWheelKraken.setVoltage(volts);
    }

    @Override
    public void setFrontWheelAmps(double amps) {
        frontWheelKraken.setControl(new TorqueCurrentFOC(amps).withUpdateFreqHz(0.0));
        frontWheelKrakenFollower.setControl(new TorqueCurrentFOC(amps));
    }

    @Override
    public void setHoodWheelAmps(double amps) {
        hoodWheelKraken.setControl(new TorqueCurrentFOC(amps));
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
            // if(0.0 < (targetVelocityRPS - frontWheelKraken.getVelocity().getValueAsDouble())) {
            //     frontWheelKraken.setVoltage(11.0);
            //     frontWheelKrakenFollower.setVoltage(11.0);
            // } else {
            //     frontWheelKraken.setVoltage(0.0);
            //     frontWheelKrakenFollower.setVoltage(0.0);
            // }
            // // if error is above deadzone apply bang bang controller voltage
            // if(bangBangControllerDeadzoneMPS < Math.abs(targetFrontWheelMPSLocal - 
            //     frontWheelKraken.getVelocity().getValueAsDouble()* (Math.PI*TurretConstants.mainFlywheelDiameterMeters))) {
                
            //     frontWheelKraken.setVoltage(bangBangControllerVolts);
            //     frontWheelKrakenFollower.setVoltage(bangBangControllerVolts);
            // } else {
            //     frontWheelKraken.setControl(velocityVoltage.withVelocity(targetVelocityRPS));
            //     frontWheelKrakenFollower.setControl(new Follower(TurretConstants.frontWheelKrakenID, MotorAlignmentValue.Opposed));
            // }
        }
    }

    @Override
    public void setTargetHoodWheelVelocity(double targetVelocityRPS) {
        targetHoodWheelMPSLocal = targetVelocityRPS*1.0 * (Math.PI*TurretConstants.hoodFlywheelDiameterMeters);

        if(!(runningBangBangController)) {
            hoodWheelKraken.setControl(velTorqueFOC.withVelocity(targetVelocityRPS*1.0));
        } else {
            hoodWheelKraken.setControl(velocityVoltage.withVelocity(targetVelocityRPS));
            // if(0.0 < (targetVelocityRPS - hoodWheelKraken.getVelocity().getValueAsDouble())) {
            //     // hoodWheelKraken.setControl(bangBangVoltage.withVelocity(targetVelocityRPS));
            //     hoodWheelKraken.setVoltage(11.0);
            // } else {
            //     hoodWheelKraken.setVoltage(0.0);
            // }
            // // if error is above deadzone apply bang bang controller voltage
            // if(bangBangControllerDeadzoneMPS < Math.abs(targetFrontWheelMPSLocal - 
            //     hoodWheelKraken.getVelocity().getValueAsDouble()* (Math.PI*TurretConstants.hoodFlywheelDiameterMeters))) {
                
            //     hoodWheelKraken.setVoltage(bangBangControllerVolts);
            // } else {
            //     hoodWheelKraken.setControl(velocityVoltage.withVelocity(targetVelocityRPS));
            // }
        }
    }
    
}
