package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;

public class IndexerIOKraken implements IndexerIO {

    private VelocityTorqueCurrentFOC velTorqueFOCRequest = new VelocityTorqueCurrentFOC(0.0).withSlot(0)
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
        config.CurrentLimits.StatorCurrentLimit = 75;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 2.42;
        config.Slot0.kP = 120.0; 
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.0;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 70;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -70;
        config.TorqueCurrent.TorqueNeutralDeadband = 0.0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = IndexerConstants.bigSpinnerGearRatio;
        config.ClosedLoopGeneral.ContinuousWrap = true;
        bigSpinnerKraken.applyConfig(config);
    }

    private void configSideKickerKraken() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 75;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 5.2;
        config.Slot0.kP = 13.0; 
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.0;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 70;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -70;
        config.TorqueCurrent.TorqueNeutralDeadband = 0.0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = IndexerConstants.sideKickerGearRatio;
        config.ClosedLoopGeneral.ContinuousWrap = true;
        sideKickerKraken.applyConfig(config);
    }

    private void configKickerKraken() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 75;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 9.3;
        config.Slot0.kP = 17.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.0;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 70;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -70;
        config.TorqueCurrent.TorqueNeutralDeadband = 0.0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = IndexerConstants.kickerGearRatio;
        config.ClosedLoopGeneral.ContinuousWrap = true;
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
        kickerKraken.setControl(velTorqueFOCRequest.withVelocity(targetVelocityRPS));
    }
}
