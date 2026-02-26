package frc.robot.subsystems.turret.flywheels;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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

    private double targetFrontWheelRPSLocal = 0.0;
    private double targetHoodWheelRPSLocal = 0.0;

    private VelocityTorqueCurrentFOC velTorqueFOC = new VelocityTorqueCurrentFOC(0.0).withSlot(0)
        .withUpdateFreqHz(0.0);

    public FlywheelsIOKraken() {
        frontWheelKraken = new Kraken(TurretConstants.frontWheelKrakenID, UniversalConstants.canivoreName);
        frontWheelKrakenFollower = new Kraken(TurretConstants.frontWheelFollowerKrakenID, UniversalConstants.canivoreName);
        hoodWheelKraken = new Kraken(TurretConstants.hoodWheelKrakenID, UniversalConstants.canivoreName);

        configFrontWheelKrakens();
        configHoodWheelKrakens();
    }

    private void configFrontWheelKrakens() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 70;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 0.0;
        config.Slot0.kP = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.0;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 70;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -70;
        config.TorqueCurrent.TorqueNeutralDeadband = 0.0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = TurretConstants.mainWheelKrakenToTurretRotationsGearRatio;
        config.ClosedLoopGeneral.ContinuousWrap = true;
        frontWheelKraken.applyConfig(config);

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        frontWheelKrakenFollower.applyConfig(config);
    }

    private void configHoodWheelKrakens() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 70;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 0.0;
        config.Slot0.kP = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.0;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 70;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -70;
        config.TorqueCurrent.TorqueNeutralDeadband = 0.0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = TurretConstants.hoodWheelKrakenToTurretRotationsGearRatio;
        config.ClosedLoopGeneral.ContinuousWrap = true;
        hoodWheelKraken.applyConfig(config);
    }

    public void updateInputs(FlywheelsIOInputs inputs) {
        inputs.frontWheelVelocityRPS = frontWheelKraken.getVelocity().getValueAsDouble();
        inputs.hoodWheelVelocityRPS = hoodWheelKraken.getVelocity().getValueAsDouble();

        inputs.frontWheelVelocityMPS = inputs.frontWheelVelocityRPS*(Math.PI*TurretConstants.mainFlywheelDiameterMeters);
        inputs.hoodWheelVelocityMPS = inputs.hoodWheelVelocityRPS*(Math.PI*TurretConstants.hoodFlywheelDiameterMeters);

        inputs.targetFrontWheelVelocityMPS = targetFrontWheelRPSLocal;
        inputs.targetHoodWheelVelocityMPS = targetHoodWheelRPSLocal;

        inputs.frontWheelAppliedVoltage = frontWheelKraken.getMotorVoltage().getValueAsDouble();
        inputs.frontWheelFollowerAppliedVoltage = frontWheelKrakenFollower.getMotorVoltage().getValueAsDouble();
        inputs.hoodWheelAppledVoltage = hoodWheelKraken.getMotorVoltage().getValueAsDouble();

        inputs.frontWheelAmps = frontWheelKraken.getStatorCurrent().getValueAsDouble();
        inputs.frontWheelFollowerAmps= frontWheelKrakenFollower.getStatorCurrent().getValueAsDouble();
        inputs.hoodWheelAmps = hoodWheelKraken.getStatorCurrent().getValueAsDouble();
    }

    public void setFrontWheelVolts(double volts) {
        frontWheelKraken.setVoltage(volts);
        frontWheelKrakenFollower.setVoltage(volts);
    }

    public void setHoodWheelVolts(double volts) {
        hoodWheelKraken.setVoltage(volts);
    }

    public void setFrontWheelAmps(double amps) {
        frontWheelKraken.setControl(new TorqueCurrentFOC(amps).withUpdateFreqHz(0.0));
        frontWheelKrakenFollower.setControl(new TorqueCurrentFOC(amps));
    }

    public void setHoodWheelAmps(double amps) {
        hoodWheelKraken.setControl(new TorqueCurrentFOC(amps));
    }

    public void setTargetFrontWheelVelocity(double targetVelocityRPS) {
        targetFrontWheelRPSLocal = targetVelocityRPS;
        frontWheelKraken.setControl(velTorqueFOC.withVelocity(targetVelocityRPS));
        // even is you set the motors to clockwise and counterclockwise you still need to set opposed if opposed
        frontWheelKrakenFollower.setControl(new Follower(TurretConstants.frontWheelKrakenID, MotorAlignmentValue.Opposed));
    }

    public void setTargetHoodWheelVelocity(double targetVelocityRPS) {
        targetHoodWheelRPSLocal = targetVelocityRPS;
        hoodWheelKraken.setControl(velTorqueFOC.withVelocity(targetVelocityRPS));
    }
    
}
