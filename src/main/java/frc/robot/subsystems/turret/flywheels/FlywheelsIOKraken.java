package frc.robot.subsystems.turret.flywheels;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;

public class FlywheelsIOKraken implements FlywheelsIO {
    private Kraken frontWheelKraken;
    private Kraken frontWheelKrakenFollower;
    private Kraken hoodWheelKraken;

    private VelocityTorqueCurrentFOC velTorqueFOC = new VelocityTorqueCurrentFOC(0.0).withSlot(2)
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
        config.CurrentLimits.StatorCurrentLimit = 75;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 0.0;
        config.Slot0.kP = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.0;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 70;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -70;

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
        config.CurrentLimits.StatorCurrentLimit = 75;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 0.0;
        config.Slot0.kP = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.0;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 70;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -70;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = TurretConstants.hoodWheelKrakenToTurretRotationsGearRatio;
        config.ClosedLoopGeneral.ContinuousWrap = true;
        hoodWheelKraken.applyConfig(config);
    }
    
}
