package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;

public class IndexerIOKraken implements IndexerIO {

    private VelocityTorqueCurrentFOC velTorqueFOCRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(0)
        .withUpdateFreqHz(0.0);

    private VelocityVoltage velVoltage = new VelocityVoltage(0.0).withSlot(1)
        .withUpdateFreqHz(0.0);

    private Kraken bigSpinnerKraken;
    private Kraken sideKickerKraken;
    private Kraken kickerKraken;

    private double bigSpinnerTargetRPSLocal = 0.0;
    private double sideKickerTargetRPSLocal = 0.0;
    private double kickerTargetRPSLocal = 0.0;

    public IndexerIOKraken() {
        bigSpinnerKraken = new Kraken(IndexerConstants.bigSpinnerID, UniversalConstants.canivoreName);
        sideKickerKraken = new Kraken(IndexerConstants.sideKickerID, UniversalConstants.canivoreName);
        kickerKraken = new Kraken(IndexerConstants.kickerID, UniversalConstants.canivoreName);

        configBigSpinnerKraken();
        configSideKickerKraken();
        configKickerKraken();
    }
    
    private void configBigSpinnerKraken() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 100;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 3.2;
        config.Slot0.kP = 110.0; 
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.0;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 100;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -100;
        config.TorqueCurrent.TorqueNeutralDeadband = 0.0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = IndexerConstants.bigSpinnerGearRatio;
        bigSpinnerKraken.applyConfig(config);
    }

    private void configSideKickerKraken() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 5.2;
        config.Slot0.kP = 13.5; 
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.0;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        config.TorqueCurrent.TorqueNeutralDeadband = 0.0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = IndexerConstants.sideKickerGearRatio;
        sideKickerKraken.applyConfig(config);
    }

    private void configKickerKraken() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 9.3;
        config.Slot0.kP = 17.5;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.0;

        config.Slot1.kS = 0.80861;
        config.Slot1.kV = 0.17337;
        // config.Slot1.kP = 99999999.1;

        config.Voltage.PeakForwardVoltage = 11.0;
        config.Voltage.PeakReverseVoltage = 0.0;

        // config.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        // config.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        // config.TorqueCurrent.TorqueNeutralDeadband = 0.0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = IndexerConstants.kickerGearRatio;
        kickerKraken.applyConfig(config);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.bigSpinnerVelocityRPS = bigSpinnerKraken.getVelocity().getValueAsDouble();
        inputs.bigSpinnerTargetRPS = bigSpinnerTargetRPSLocal;
        inputs.bigSpinnerVolts = bigSpinnerKraken.getMotorVoltage().getValueAsDouble();
        inputs.bigSpinnerAmps = bigSpinnerKraken.getStatorCurrent().getValueAsDouble();

        inputs.sideKickerVelocityRPS = sideKickerKraken.getVelocity().getValueAsDouble();
        inputs.sideKickerTargetRPS = sideKickerTargetRPSLocal;
        inputs.sideKickerVolts = sideKickerKraken.getMotorVoltage().getValueAsDouble();
        inputs.sideKickerAmps = sideKickerKraken.getStatorCurrent().getValueAsDouble();

        inputs.kickerVelocityRPS = kickerKraken.getVelocity().getValueAsDouble();
        inputs.kickerTargetRPS = kickerTargetRPSLocal;
        inputs.kickerVolts = kickerKraken.getMotorVoltage().getValueAsDouble();
        inputs.kickerAmps = kickerKraken.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setBigSpinnerVolts(double volts) {
        bigSpinnerKraken.setVoltage(volts);
    }

    @Override
    public void setSideKickerVolts(double volts) {
        sideKickerKraken.setVoltage(volts);
    }

    @Override
    public void setKickerVolts(double volts) {
        kickerKraken.setVoltage(volts);
    }

    @Override
    public void setBigSpinnerAmps(double amps) {
        bigSpinnerKraken.setControl(new TorqueCurrentFOC(amps).withUpdateFreqHz(0.0));
    }

    @Override
    public void setSideKickerAmps(double amps) {
        sideKickerKraken.setControl(new TorqueCurrentFOC(amps).withUpdateFreqHz(0.0));
    }

    @Override
    public void setKickerAmps(double amps) {
        kickerKraken.setControl(new TorqueCurrentFOC(amps).withUpdateFreqHz(0.0));
    }

    @Override
    public void setTargetBigSpinnerVelocity(double targetVelocityRPS) {
        bigSpinnerTargetRPSLocal = targetVelocityRPS;
        bigSpinnerKraken.setControl(velTorqueFOCRequest.withVelocity(targetVelocityRPS));
    }

    @Override  
    public void setTargetSideKickerVelocity(double targetVelocityRPS) {
        sideKickerTargetRPSLocal = targetVelocityRPS;
        sideKickerKraken.setControl(velTorqueFOCRequest.withVelocity(targetVelocityRPS));
    }

    @Override
    public void setTargetKickerVelocity(double targetVelocityRPS) {
        kickerTargetRPSLocal = targetVelocityRPS;
        kickerKraken.setControl(velVoltage.withVelocity(targetVelocityRPS));
    }
}
