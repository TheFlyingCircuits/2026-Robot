package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;

public class AimerIOKraken implements AimerIO{
    
    private Kraken aimerKraken;

    public AimerIOKraken() {
        aimerKraken = new Kraken(TurretConstants.aimerKrakenID, UniversalConstants.canivoreName);
        configAimerKraken();
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
        config.ClosedLoopGeneral.ContinuousWrap = false;
        aimerKraken.applyConfig(config);
    }

}
