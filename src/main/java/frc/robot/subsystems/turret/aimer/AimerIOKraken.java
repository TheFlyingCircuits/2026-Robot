package frc.robot.subsystems.turret.aimer;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AimerIOKraken implements AimerIO{
    
    private Kraken aimerKraken;
    private CANcoder absoluteEncoder;

    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(1).withEnableFOC(true)
        .withUpdateFreqHz(0.0);

    // DO NOT use anything besides get pose meters we don't want conflicts
    private Drivetrain drivetrain;

    private double targetAimerDegrees = 0.0;


    public AimerIOKraken(Drivetrain drivetrain) {
        // aimerKraken = new Kraken(TurretConstants.aimerKrakenID, UniversalConstants.canivoreName);
        absoluteEncoder = new CANcoder(TurretConstants.aimerCANcoderID, UniversalConstants.canivoreName);
        aimerKraken = new Kraken(TurretConstants.aimerKrakenID, UniversalConstants.canivoreName);
        configCANCoder();
        configAimerKraken();
        this.drivetrain=drivetrain;
    }

    private void configCANCoder() {
        CANcoderConfiguration cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        cancoderConfigs.MagnetSensor.MagnetOffset = TurretConstants.aimerCANcoderOffset;
        cancoderConfigs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        absoluteEncoder.getConfigurator().apply(cancoderConfigs);
    }


    private void configAimerKraken() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.StatorCurrentLimit = 25;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 0.0; 
        config.Slot0.kP = 0.0;
        config.Slot0.kI = 0.0; 
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.285;

        config.Slot1.kS = 0.22;
        config.Slot1.kP = 0.0; 

        config.MotionMagic.MotionMagicCruiseVelocity = 1000.0; //rps
        config.MotionMagic.MotionMagicAcceleration = 200.0; //rotations per second squared
        // Units.degreesToRotations(1100);
        config.ClosedLoopGeneral.GainSchedErrorThreshold = Units.degreesToRotations(0.3);

        double encoderGearAngleRotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.FeedbackRemoteSensorID = TurretConstants.aimerCANcoderID;
        config.Feedback.RotorToSensorRatio = TurretConstants.aimerKrakenRotorToCANcoderGearRatio;
        config.Feedback.SensorToMechanismRatio = TurretConstants.aimerKrakenToTurretRotationsGearRatio;
        config.ClosedLoopGeneral.ContinuousWrap = false;

        aimerKraken.applyConfig(config);

        // 0.16875 of mechanism = 1 rotation of cancoder gear so this is also the range that it can see when init
        // since range is -0.5 to 0.5 we do minTurretAngle + (0.16875/2.0 * canconder angle in rotations) is the initial turret position
        double turretsInitialAngleRot = TurretConstants.turretMinAngle + ((0.16875/2.0) * encoderGearAngleRotations);

        aimerKraken.setPosition(turretsInitialAngleRot);
    }

    @Override
    public void updateInputs(AimerIOInputs inputs) {
        inputs.aimerPositionDegrees = Units.rotationsToDegrees(aimerKraken.getPosition().getValueAsDouble());
        inputs.aimerVelocityDegreesPerSecond = Units.rotationsToDegrees(aimerKraken.getVelocity().getValueAsDouble());
        inputs.aimerTargetPositionDegrees = targetAimerDegrees;

        inputs.aimerAppliedVoltage = aimerKraken.getMotorVoltage().getValueAsDouble();
        inputs.aimerAmps = aimerKraken.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setAimerVolts(double volts) {
        aimerKraken.setVoltage(volts);//2200/3300
    }

    @Override
    public void setTargetAimerPosition(double targetPositionDegreesRobotToTarget) {
        if(targetPositionDegreesRobotToTarget > 175.0) {
            targetPositionDegreesRobotToTarget = 175.0;
        } else if(targetPositionDegreesRobotToTarget < -175.0) {
            targetPositionDegreesRobotToTarget = -175.0;
        }
        double targetAngleDegreesTurretToTarget = targetPositionDegreesRobotToTarget - drivetrain.getPoseMeters().getRotation().getDegrees();
        targetAimerDegrees=targetAngleDegreesTurretToTarget;
        if(Math.abs(targetAngleDegreesTurretToTarget-(Units.rotationsToDegrees(aimerKraken.getPosition().getValueAsDouble()))) > 7.5) {
            aimerKraken.setControl(new MotionMagicVoltage(Units.degreesToRotations(targetAngleDegreesTurretToTarget)).withEnableFOC(true)
        .withUpdateFreqHz(0.0)
        );
        } else {
            aimerKraken.setControl(m_request.withPosition(Units.degreesToRotations(targetAngleDegreesTurretToTarget)));
        }
        // aimerKraken.setControl(m_Velrequest.withVelocity(Units.degreesToRotations(targetPositionDegrees)));
    }

}
