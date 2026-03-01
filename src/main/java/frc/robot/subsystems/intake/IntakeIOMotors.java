package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;
import frc.robot.VendorWrappers.Neo;


public class IntakeIOMotors implements IntakeIO{
    private Neo pivotNeo;
    private Kraken rollerTopKraken;
    private Kraken rollerBottomKraken;

    final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true)
        .withUpdateFreqHz(0.0);
    final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true)
        .withUpdateFreqHz(0.0);

    public IntakeIOMotors() {
       pivotNeo = new Neo(IntakeConstants.pivotNeoID);
       rollerTopKraken = new Kraken(IntakeConstants.rollerTopKrakenID, UniversalConstants.canivoreName);
       rollerBottomKraken = new Kraken(IntakeConstants.rollerBottomKrakenID, UniversalConstants.canivoreName);
       configPivotNeo();
       configRollerTopKraken();
       configRollerBottomKraken();
    }

    private void configPivotNeo() {
        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        pivotConfig.idleMode(IdleMode.kCoast);
        pivotConfig.smartCurrentLimit(40);
        pivotConfig.inverted(true);
        pivotConfig.closedLoop.positionWrappingEnabled(false);

        // account for gear ratio I think its 42:1 then convert rotations to deg by * 360
        pivotConfig.encoder.positionConversionFactor((1.0/42.0)*360);

        pivotNeo.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotNeo.getEncoder().setPosition(IntakeConstants.intakeStartDegrees);
    }
  
    private void configRollerTopKraken() {
       TalonFXConfiguration config = new TalonFXConfiguration();
       config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
       config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       config.CurrentLimits.StatorCurrentLimit = 25;
       config.CurrentLimits.StatorCurrentLimitEnable = true;

       config.Slot0.kS = 0.36;
       config.Slot0.kV = 8.25;
       config.Slot0.kP = 3.0;
       config.Slot0.kI = 0.0;
       config.Slot0.kD = 0.0;
       config.ClosedLoopGeneral.ContinuousWrap = true;
       config.Feedback.RotorToSensorRatio = 2.0;
       rollerTopKraken.applyConfig(config);
    }


    private void configRollerBottomKraken() {
       TalonFXConfiguration config = new TalonFXConfiguration();
       config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
       config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       config.CurrentLimits.StatorCurrentLimit = 25;
       config.CurrentLimits.StatorCurrentLimitEnable = true;

       config.Slot0.kS = 0.53;
       config.Slot0.kV = 9.9;
       config.Slot0.kP = 3.0;
       config.Slot0.kI = 0.0;
       config.Slot0.kD = 0.0;
       config.Feedback.RotorToSensorRatio = 1.25;
       config.ClosedLoopGeneral.ContinuousWrap = true;
       rollerBottomKraken.applyConfig(config);
    }
      
    @Override
    public void setRollerTopVolts(double volts) {
        rollerTopKraken.setVoltage(volts);
    }

    @Override
    public void setRollerBottomVolts(double volts) {
        rollerBottomKraken.setVoltage(volts);
    }

    @Override
    public void setIntakeVolts(double volts) {
        pivotNeo.setVoltage(volts);
    }

    @Override
    public void setTargetRollerTopVelocity(double velocityRPS) {
        rollerTopKraken.setControl(velocityRequest.withVelocity(velocityRPS));
    }

    @Override
    public void setTargetRollerBottomVelocity(double velocityRPS) {
        rollerBottomKraken.setControl(velocityRequest.withVelocity(velocityRPS));
    }

    @Override
    public void setTargetIntakePositionDegrees(double targetDegrees) {
        double error = targetDegrees - pivotNeo.getPosition();
        double outputVolts =  error * IntakeConstants.kP + IntakeConstants.kS;
        pivotNeo.setVoltage(outputVolts);
    }
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeVelocityRPS = pivotNeo.getVelocity();
        inputs.rollerTopVelocityRPS = rollerTopKraken.getVelocity().getValueAsDouble();
        inputs.rollerBottomVelocityRPS = rollerBottomKraken.getVelocity().getValueAsDouble();
        inputs.intakePositionDegrees = pivotNeo.getAbsoluteEncoder().getPosition();
        
        inputs.intakeVolts = pivotNeo.getAppliedOutput()*pivotNeo.getBusVoltage();
        inputs.rollerTopVolts = rollerTopKraken.getMotorVoltage().getValueAsDouble();
        inputs.rollerBottomVolts = rollerBottomKraken.getMotorVoltage().getValueAsDouble();

        inputs.intakeAmps = pivotNeo.getOutputCurrent();
        inputs.rollerTopAmps = rollerTopKraken.getStatorCurrent().getValueAsDouble();
        inputs.rollerBottomAmps = rollerBottomKraken.getStatorCurrent().getValueAsDouble();
    }
}
