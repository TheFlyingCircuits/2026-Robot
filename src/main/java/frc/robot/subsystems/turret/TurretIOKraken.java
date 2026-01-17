package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;

public class TurretIOKraken implements TurretIO {

    private Kraken aimerKraken;
    private Kraken frontWheelKraken;
    private Kraken hoodWheelKraken;
    private Kraken hoodKraken;

    public TurretIOKraken() {

        aimerKraken = new Kraken(TurretConstants.aimerKrakenID, UniversalConstants.canivoreName);
        frontWheelKraken = new Kraken(TurretConstants.frontWheelKrakenID, UniversalConstants.canivoreName);
        hoodWheelKraken = new Kraken(TurretConstants.hoodWheelKrakenID, UniversalConstants.canivoreName);
        hoodKraken = new Kraken(TurretConstants.hoodKrakenID, UniversalConstants.canivoreName);

        configAimerKraken();
        configFrontWheelKraken();
        configHoodWheelKraken();
        configHoodKraken();
    }

    private void configAimerKraken() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 25;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 0.0;
        config.Slot0.kP = 0.0;
        config.Slot0.kI = 0.0; 
        config.Slot0.kD = 0.0;
        config.Feedback.SensorToMechanismRatio = 1;
        config.ClosedLoopGeneral.ContinuousWrap = false;
        aimerKraken.applyConfig(config);
    }

    private void configFrontWheelKraken() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 25; // re-determined after firmware upgrade to prevent wheel slip. Feels pretty low though

        config.Slot1.kS = 0.0; 
        config.Slot1.kV = 0.0;
        config.Slot1.kP = 0.0;
        config.Slot1.kI = 0.0;
        config.Slot1.kD = 0.0;
        frontWheelKraken.applyConfig(config);
    }

    private void configHoodWheelKraken() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 25; // re-determined after firmware upgrade to prevent wheel slip. Feels pretty low though

        config.Slot1.kS = 0.0; 
        config.Slot1.kV = 0.0;
        config.Slot1.kP = 0.0;
        config.Slot1.kI = 0.0;
        config.Slot1.kD = 0.0;
        hoodWheelKraken.applyConfig(config);
    }

    private void configHoodKraken() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 25;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 0.0;
        config.Slot0.kP = 0.0;
        config.Slot0.kI = 0.0; 
        config.Slot0.kD = 0.0;
        config.Feedback.SensorToMechanismRatio = 1;
        config.ClosedLoopGeneral.ContinuousWrap = false;
        hoodKraken.applyConfig(config);
    }
    

}
