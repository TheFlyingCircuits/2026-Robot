package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
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
    final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
    final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);


    public IntakeIOKraken() {
       intakeLeftKraken = new Kraken(IntakeConstants.intakeLeftKrakenID, UniversalConstants.canivoreName);
       intakeRightKraken = new Kraken(IntakeConstants.intakeRightKrakenID, UniversalConstants.canivoreName);
       rollerTopKraken = new Kraken(IntakeConstants.rollerTopKrakenID, UniversalConstants.canivoreName);
       rollerBottomKraken = new Kraken(IntakeConstants.rollerBottomKrakenID, UniversalConstants.canivoreName);
       intakeCANcoder = new CANcoder(IntakeConstants.leftPivotEncoderID, UniversalConstants.canivoreName);
       configIntakeLeftKraken();
       configIntakeRightKraken();
       configRollerTopKraken();
       configRollerBottomKraken();
    }


    private void configIntakeLeftKraken() {
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
       intakeLeftKraken.applyConfig(config);
    }


    private void configIntakeRightKraken() {
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
      
    public void setRollerTopVolts(double volts) {
        rollerTopKraken.setVoltage(volts);
    };


    public void setRollerBottomVolts(double volts) {
        rollerBottomKraken.setVoltage(volts);
    };


    public void setIntakeVolts(double volts) {
        intakeLeftKraken.setVoltage(volts);
    };


    public void setTargetRollerTopVelocity(double volts) {
        rollerTopKraken.setControl(velocityRequest.withAcceleration(volts));
    };


    public void setTargetRollerBottomVelocity(double volts) {
        rollerBottomKraken.setControl(velocityRequest.withAcceleration(volts));
    };


    public void setTargetIntakePositionDegrees(double degrees) {
        intakeLeftKraken.setControl(positionRequest.withPosition(degrees));
    };
    
   @Override
   public void updateInputs(IntakeIOInputs inputs) {
       inputs.intakeVelocityRPM = intakeLeftKraken.getVelocity().getValueAsDouble() * 60;
       inputs.rollerTopVelocityRPM = rollerTopKraken.getVelocity().getValueAsDouble() * 60;
       inputs.rollerBottomVelocityRPM = rollerBottomKraken.getVelocity().getValueAsDouble() * 60;
       inputs.intakePositionDegrees = intakeCANcoder.getPosition().getValueAsDouble();
       inputs.intakeVelocityDegreesPerSecond = intakeCANcoder.getVelocity().getValueAsDouble();
   }
}
