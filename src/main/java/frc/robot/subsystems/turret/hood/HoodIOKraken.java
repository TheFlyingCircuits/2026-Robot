package frc.robot.subsystems.turret.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;

public class HoodIOKraken implements HoodIO{
    private Kraken hoodKraken;
    private double targetHoodDegreesLocal = 0.0;

    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0).withEnableFOC(true)
        .withUpdateFreqHz(0.0);

    public HoodIOKraken() {
        hoodKraken = new Kraken(TurretConstants.hoodKrakenID, UniversalConstants.canivoreName);
        configHoodKraken();
    }

    private void configHoodKraken() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 35;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 0.6; // Add 0.6 V output to overcome static friction
        config.Slot0.kP = 90.0; // An error of 1 rotation results in 20.0 V output for each 1/8th rot off 2.5V
        config.Slot0.kI = 0.0; 
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.0;


        // config.ClosedLoopGeneral.GainSchedErrorThreshold = Units.degreesToRotations(0.2);
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = TurretConstants.hoodKrakenToTurretRotationsGearRatio;
        config.ClosedLoopGeneral.ContinuousWrap = false;
        hoodKraken.applyConfig(config);
        hoodKraken.setPosition(Units.degreesToRotations(TurretConstants.maxHoodAngle));
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodPositionDegrees = Units.rotationsToDegrees(hoodKraken.getPosition().getValueAsDouble());
        inputs.hoodVelocityDegreesPerSecond = Units.rotationsToDegrees(hoodKraken.getVelocity().getValueAsDouble());
        inputs.targetHoodPositionDegrees = targetHoodDegreesLocal;

        inputs.hoodAppliedVoltage = hoodKraken.getMotorVoltage().getValueAsDouble();
        inputs.hoodAmps = hoodKraken.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setHoodVolts(double volts) {
        hoodKraken.setVoltage(volts);
    }

    @Override
    public void setTargetHoodPosition(double targetPositionDegrees) {
        if(targetPositionDegrees < TurretConstants.minHoodAngle + 1.0) {
            targetPositionDegrees = TurretConstants.minHoodAngle + 1.0;
        } else if(targetPositionDegrees > TurretConstants.maxHoodAngle -1.0) {
            targetPositionDegrees = TurretConstants.maxHoodAngle - 1.0;
        }
        targetHoodDegreesLocal = targetPositionDegrees;
        hoodKraken.setControl(m_request.withPosition(Units.degreesToRotations(targetPositionDegrees)));
    }

}