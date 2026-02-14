package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;


public class IntakeIOKraken implements IntakeIO{
    private Kraken intakeLeftKraken;
    private Kraken intakeRightKraken;
    private Kraken rollerTopKraken;
    private Kraken rollerBottomKraken;
    private CANcoder intakeCANcoder;
    final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true)
        .withUpdateFreqHz(0.0);
    final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true)
        .withUpdateFreqHz(0.0);

    public IntakeIOKraken() {
       intakeLeftKraken = new Kraken(IntakeConstants.intakeLeftKrakenID, UniversalConstants.canivoreName);
       intakeRightKraken = new Kraken(IntakeConstants.intakeRightKrakenID, UniversalConstants.canivoreName);
       rollerTopKraken = new Kraken(IntakeConstants.rollerTopKrakenID, UniversalConstants.canivoreName);
       rollerBottomKraken = new Kraken(IntakeConstants.rollerBottomKrakenID, UniversalConstants.canivoreName);
       intakeCANcoder = new CANcoder(IntakeConstants.leftPivotEncoderID, UniversalConstants.canivoreName);
       configIntakeKrakens();
       configRollerTopKraken();
       configRollerBottomKraken();
    }


    private void configIntakeKrakens() {
       TalonFXConfiguration config = new TalonFXConfiguration();
       config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
       config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
       config.CurrentLimits.StatorCurrentLimit = 25;
       config.CurrentLimits.StatorCurrentLimitEnable = true;


       config.Slot0.kS = 0.0;
       config.Slot0.kP = 0.0;
       config.Slot0.kI = 0.0;
       config.Slot0.kD = 0.0;

       config.ClosedLoopGeneral.ContinuousWrap = false;
       config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; //ask simon abt this
       config.Feedback.FeedbackRemoteSensorID = 0;
       config.Feedback.SensorToMechanismRatio = 0;
       intakeLeftKraken.applyConfig(config);
       intakeRightKraken.applyConfig(config);
       intakeRightKraken.setControl(new Follower(intakeLeftKraken.getDeviceID(), MotorAlignmentValue.Aligned));
    }
  
    private void configRollerTopKraken() {
       TalonFXConfiguration config = new TalonFXConfiguration();
       config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
       config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       config.CurrentLimits.StatorCurrentLimit = 25;
       config.CurrentLimits.StatorCurrentLimitEnable = true;


       config.Slot0.kS = 0.0;
       config.Slot0.kP = 0.0;
       config.Slot0.kI = 0.0;
       config.Slot0.kD = 0.0;
       config.ClosedLoopGeneral.ContinuousWrap = true;
       rollerTopKraken.applyConfig(config);
    }


    private void configRollerBottomKraken() {
       TalonFXConfiguration config = new TalonFXConfiguration();
       config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
       config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       config.CurrentLimits.StatorCurrentLimit = 25;
       config.CurrentLimits.StatorCurrentLimitEnable = true;


       config.Slot0.kS = 0.0;
       config.Slot0.kP = 0.0;
       config.Slot0.kI = 0.0;
       config.Slot0.kD = 0.0;
       config.ClosedLoopGeneral.ContinuousWrap = true;
       rollerBottomKraken.applyConfig(config);
    }
      
    @Override
    public void setRollerTopVolts(double volts) {
        rollerTopKraken.setVoltage(volts);
    };

    @Override
    public void setRollerBottomVolts(double volts) {
        rollerBottomKraken.setVoltage(volts);
    };

    @Override
    public void setIntakeVolts(double volts) {
        intakeLeftKraken.setVoltage(volts);
    };

    @Override
    public void setTargetRollerTopVelocity(double velocity) {
        rollerTopKraken.setControl(velocityRequest.withVelocity(velocity));
    };

    @Override
    public void setTargetRollerBottomVelocity(double velocity) {
        rollerBottomKraken.setControl(velocityRequest.withVelocity(velocity));
    };

    @Override
    public void setTargetIntakePositionDegrees(double degrees) {
        intakeLeftKraken.setControl(positionRequest.withPosition(degrees));
    };
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeVelocityRPS = intakeLeftKraken.getVelocity().getValueAsDouble();
        inputs.rollerTopVelocityRPS = rollerTopKraken.getVelocity().getValueAsDouble();
        inputs.rollerBottomVelocityRPS = rollerBottomKraken.getVelocity().getValueAsDouble();
        inputs.intakePositionDegrees = intakeCANcoder.getPosition().getValueAsDouble();
        inputs.intakeVelocityDegreesPerSecond = intakeCANcoder.getVelocity().getValueAsDouble();
    }
}
