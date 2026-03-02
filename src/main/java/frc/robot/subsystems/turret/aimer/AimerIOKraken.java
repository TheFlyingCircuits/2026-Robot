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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AimerIOKraken implements AimerIO{
    
    private Kraken aimerKraken;
    private CANcoder absoluteEncoder;
    private double turretSpringAngleRobotRelative = 0.0;
    // private double turretMaxRobotRelativeDeg = new Rotation2d(Units.degreesToRadians(turretZeroDegreesRobotRelative)).plus(Rotation2d.k180deg).getDegrees();
    private double ksForConstantForceSpring = 0.9;

    private double turretMaxOneSideDeg = 190;// TODO: get real

    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(1).withEnableFOC(true)
        .withUpdateFreqHz(0.0).withSlot(1);

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
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 50;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 0.0; // ks will be 0 because will acount for outside of talon fx control loop
        config.Slot0.kP = 3.0;
        config.Slot0.kI = 0.0; 
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 2.5; // rps/volts

        config.Slot1.kS = 0.0;
        config.Slot1.kP = 32.0; 

        config.MotionMagic.MotionMagicCruiseVelocity = 1.5; //rps
        config.MotionMagic.MotionMagicAcceleration = 1.5; //rotations per second squared
        // Units.degreesToRotations(1100);
        // config.ClosedLoopGeneral.GainSchedErrorThreshold = Units.degreesToRotations(0.0);

        double encoderGearAnglePositionInRotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = TurretConstants.aimerCANcoderID;
        config.Feedback.RotorToSensorRatio = TurretConstants.aimerKrakenRotorToCANcoderGearRatio;
        config.Feedback.SensorToMechanismRatio = TurretConstants.canCoderToTurretRotationsGearRatio;
        config.ClosedLoopGeneral.ContinuousWrap = true;


        // will make it so turret will stay in range
        // SoftwareLimitSwitchConfigs limitConfigs = new SoftwareLimitSwitchConfigs();

        // limitConfigs.ForwardSoftLimitThreshold = turretMaxOneSideDeg / 360.0;
        // limitConfigs.ReverseSoftLimitThreshold = -turretMaxOneSideDeg / 360.0;

        // limitConfigs.ForwardSoftLimitEnable = true;
        // limitConfigs.ReverseSoftLimitEnable = true;

        // config.SoftwareLimitSwitch = limitConfigs;

        aimerKraken.applyConfig(config);

        // 0.16875 of mechanism = 1 rotation of cancoder gear so this is also the range that it can see when init
        // since range is -0.5 to 0.5 we do minTurretAngle + (0.16875/2.0 * canconder angle in rotations) is the initial turret position
        // double turretsInitialAngleRot = (TurretConstants.turretMinAngle + 0.16875/2.0) + ((0.16875/2.0) * encoderGearAnglePositionInRotations);

        double turretsInitialAngleRot = ((0.16875) * encoderGearAnglePositionInRotations);

        aimerKraken.setPosition(turretsInitialAngleRot);
    }

    private double getSafeOptimizedAngleDeg(double targetAngle) {

        double aimerAngleDeg = Units.rotationsToDegrees(aimerKraken.getPosition().getValueAsDouble());

        double optimizedAngle;

        if((targetAngle*aimerAngleDeg) < 0) {
            double signOfAimer = aimerAngleDeg > 0 ? 1.0 : -1.0;

            double targetDistanceFrom180Deg = 180.0-Math.abs(targetAngle);
            double aimerDistanceFrom180Deg = 180.0-Math.abs(aimerAngleDeg);
            optimizedAngle = aimerAngleDeg + (targetDistanceFrom180Deg + aimerDistanceFrom180Deg) * signOfAimer;

            if(Math.abs(optimizedAngle) > turretMaxOneSideDeg) optimizedAngle = optimizedAngle + (360.0*(signOfAimer * -1));
        } else {
            optimizedAngle = targetAngle;
        }

        return optimizedAngle;
    }

    @Override
    public void updateInputs(AimerIOInputs inputs) {
        inputs.aimerPositionDegrees = Units.rotationsToDegrees(aimerKraken.getPosition().getValueAsDouble());
        inputs.aimerVelocityDegreesPerSecond = Units.rotationsToDegrees(aimerKraken.getVelocity().getValueAsDouble());
        inputs.aimerVelocityRotationsPerSecond = aimerKraken.getVelocity().getValueAsDouble();
        inputs.aimerTargetPositionDegrees = targetAimerDegrees;

        inputs.closedLoopReference = Units.rotationsToDegrees(aimerKraken.getClosedLoopReference().getValueAsDouble());

        inputs.aimerAppliedVoltage = aimerKraken.getMotorVoltage().getValueAsDouble();
        inputs.aimerAmps = aimerKraken.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setAimerVolts(double volts) {
        aimerKraken.setVoltage(volts);
    }

    @Override
    public void setTargetAimerPosition(double targetPositionDegreesRobotToTarget) {

        double feedForwardsSpringVolts = ksForConstantForceSpring;

        double turretPositionRotations = aimerKraken.getPosition().getValueAsDouble();

        if(turretPositionRotations < 0) {
            feedForwardsSpringVolts = feedForwardsSpringVolts*-1.0;
            // range is 5 to -15
        }

        feedForwardsSpringVolts = 0.0;

        // if((turretPosition < 5.0) && (turretPosition > -15.0)) {
        //         feedForwardsSpringVolts = 0.0;
        // }
        

        double targetAngleDegreesTurretToTargetIfTurretWasFront = targetPositionDegreesRobotToTarget - drivetrain.getPoseMeters().getRotation().getDegrees();

        double targetAngleDeg180Clamped = Rotation2d.fromDegrees(targetAngleDegreesTurretToTargetIfTurretWasFront).getDegrees();

        // double targetAngleDegreesTurretToTarget = (new Rotation2d(Units.degreesToRadians(targetAngleDegreesTurretToTargetIfTurretWasFront)
        //     ).plus(new Rotation2d(Units.degreesToRadians(turretSpringAngleRobotRelative)))).getDegrees();


        double safeAngle = getSafeOptimizedAngleDeg(targetAngleDeg180Clamped);
        targetAimerDegrees = safeAngle;

        if(Math.abs(safeAngle-(Units.rotationsToDegrees(aimerKraken.getPosition().getValueAsDouble()))) > 7.5) {
            aimerKraken.setControl(new MotionMagicVoltage(Units.degreesToRotations(safeAngle)).withEnableFOC(true)
        .withUpdateFreqHz(0.0).withFeedForward(feedForwardsSpringVolts).withSlot(0));
        } else {
            aimerKraken.setControl(m_request.withPosition(Units.degreesToRotations(safeAngle)).withFeedForward(feedForwardsSpringVolts));
        }
        // aimerKraken.setControl(m_request.withPosition(Units.degreesToRotations(safeAngle)));
        // aimerKraken.setControl(m_Velrequest.withVelocity(Units.degreesToRotations(targetPositionDegrees)));
    }

}
