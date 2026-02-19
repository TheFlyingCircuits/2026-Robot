package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;
import frc.robot.VendorWrappers.Neo;


public class IntakeIOMotors implements IntakeIO{
    private Neo pivotNeo;
    private Kraken rollerTopKraken;
    private Kraken rollerBottomKraken;
    private CANcoder intakeCANcoder;
    private PIDController pivotPidController;
    private ArmFeedforward pivotFeedforward;

    final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0).withEnableFOC(true)
        .withUpdateFreqHz(0.0);
    final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(true)
        .withUpdateFreqHz(0.0);

    public IntakeIOMotors() {
       pivotNeo = new Neo(IntakeConstants.pivotNeoID);
       rollerTopKraken = new Kraken(IntakeConstants.rollerTopKrakenID, UniversalConstants.canivoreName);
       rollerBottomKraken = new Kraken(IntakeConstants.rollerBottomKrakenID, UniversalConstants.canivoreName);
       intakeCANcoder = new CANcoder(IntakeConstants.leftPivotEncoderID, UniversalConstants.canivoreName);
       configPivotNeo();
       configRollerTopKraken();
       configRollerBottomKraken();
    }

    private void configPivotNeo() {
        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        pivotConfig.idleMode(IdleMode.kCoast);
        pivotConfig.smartCurrentLimit(25);
        pivotConfig.inverted(true);
        pivotConfig.closedLoop.positionWrappingEnabled(false);
        pivotPidController = new PIDController(2, 0, 0);
        pivotFeedforward = new ArmFeedforward(0, 0, 0);
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
    public void setTargetRollerTopVelocity(double velocity) {
        rollerTopKraken.setControl(velocityRequest.withVelocity(velocity));
    }

    @Override
    public void setTargetRollerBottomVelocity(double velocity) {
        rollerBottomKraken.setControl(velocityRequest.withVelocity(velocity));
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
        inputs.intakePositionDegrees = intakeCANcoder.getPosition().getValueAsDouble();
        inputs.intakeVelocityDegreesPerSecond = intakeCANcoder.getVelocity().getValueAsDouble();
    }
}
