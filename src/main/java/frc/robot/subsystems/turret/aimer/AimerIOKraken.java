package frc.robot.subsystems.turret.aimer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AimerIOKraken implements AimerIO{
    
    private Kraken aimerKraken;
    private CANcoder absoluteEncoder;

    private double ksForConstantForceSpring = 0.65;
    private double kVVoltsVoltsPerRotation = 1.80594;
    private Timer timer;

    private double turretMaxOneSideDeg = 250;// TODO: get real

    MotionMagicVoltage motionMagic = new MotionMagicVoltage(Units.degreesToRotations(0.0)).withEnableFOC(true)
        .withUpdateFreqHz(60.0).withSlot(0);

    private final PIDController turretPIDToTarget = new PIDController(125.0,20.0,0.06);
    // private final PIDController turretPIDToTargetFar = new PIDController(35.0,0.0,0.0);
    private final PIDController turretPIDToSetpoint = new PIDController(1.5,0.0,0.0);

    // DO NOT use anything besides get pose meters we don't want conflicts
    private Drivetrain drivetrain;

    private double targetAimerDegrees = 0.0;


    public AimerIOKraken(Drivetrain drivetrain) {
        absoluteEncoder = new CANcoder(TurretConstants.aimerCANcoderID, UniversalConstants.canivoreName);
        aimerKraken = new Kraken(TurretConstants.aimerKrakenID, UniversalConstants.canivoreName);
        configCANCoder();
        configAimerKraken();
        timer = new Timer();
        timer.start();
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
        config.CurrentLimits.StatorCurrentLimit = 160;
        config.CurrentLimits.StatorCurrentLimitEnable = true;


        // motion magic y=1.80594x+0.78592
        // config.Slot0.kG = ksForConstantForceSpring; 
        config.Slot0.kP = 210.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 1.80594; // rps/volts 0.82 rps 2v - 1.4rps - 3v

        // config.Slot1.kG = -ksForConstantForceSpring; 
        config.Slot1.kP = 125.0;
        config.Slot1.kV = 1.80594;
        config.Slot1.kI = 0.0;
        config.Slot1.kD = 0.0;
        // config.Slot1.kV = 1.80594; 

        config.Slot2.kP = 0.0;
        config.Slot2.kD = 0.0;
        config.Slot2.kV = 1.80594;

        // close voltage feedback
        // ks is over 0.0 because in either direction 0.001 volts of feedback pid is not enough
        // to overcome spring feed forward because its kidnda an estimate
        // config.Slot1.kS = 0.1;
        // config.Slot1.kP = 195.0; 

        config.TorqueCurrent.PeakForwardTorqueCurrent = 160;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -160;
        config.TorqueCurrent.TorqueNeutralDeadband = 0.0;
        config.ClosedLoopGeneral.GainSchedErrorThreshold = Units.degreesToRotations(0.05);

        config.MotionMagic.MotionMagicCruiseVelocity = 1.5; //rps
        config.MotionMagic.MotionMagicAcceleration = 4.0; //rotations per second squared

        config.Voltage.PeakForwardVoltage = 8.0;
        config.Voltage.PeakReverseVoltage = -8.0;
        // Units.degreesToRotations(1100);
        // config.ClosedLoopGeneral.GainSchedErrorThreshold = Units.degreesToRotations(0.0);

        double encoderGearAnglePositionInRotations = absoluteEncoder.getAbsolutePosition().getValueAsDouble();

        // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        // config.Feedback.FeedbackRemoteSensorID = TurretConstants.aimerCANcoderID;
        // config.Feedback.RotorToSensorRatio = TurretConstants.aimerKrakenRotorToCANcoderGearRatio;
        // config.Feedback.SensorToMechanismRatio = TurretConstants.canCoderToTurretRotationsGearRatio;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        config.Feedback.SensorToMechanismRatio = TurretConstants.aimerKrakenRotorToCANcoderGearRatio*TurretConstants.canCoderToTurretRotationsGearRatio;
        config.ClosedLoopGeneral.ContinuousWrap = false;

        aimerKraken.applyConfig(config);

        // 0.16875 of mechanism = 1 rotation of cancoder gear so this is also the range that it can see when init
        // since range is -0.5 to 0.5 we do minTurretAngle + (0.16875/2.0 * canconder angle in rotations) is the initial turret position
        // double turretsInitialAngleRot = (TurretConstants.turretMinAngle + 0.16875/2.0) + ((0.16875/2.0) * encoderGearAnglePositionInRotations);

        double turretsInitialAngleRot = ((0.16875) * encoderGearAnglePositionInRotations);

        aimerKraken.setPosition(turretsInitialAngleRot);

        aimerKraken.getPosition().setUpdateFrequency(500);
        aimerKraken.getVelocity().setUpdateFrequency(500);
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
    public void setAimerAmps(double amps) {
        aimerKraken.setControl(new TorqueCurrentFOC(amps).withUpdateFreqHz(0.0));
    }

    // returns a velocity setpoint
    public double motionProfile(double currentError, double currentVelocityRotationsPerSec, 
        double maxVelocityRotationsPerSec, double maxAcelRotationsPerSecSquared) {
        // How much distance do we need to stop from current velocity
        double stoppingDistanceRot = (currentVelocityRotationsPerSec * currentVelocityRotationsPerSec) / (2.0 * maxAcelRotationsPerSecSquared);
        
        // If we need to start slowing down to stop in time
        if (Math.abs(currentError) <= stoppingDistanceRot) {
            // Attempt to decelerate if we are at or under our stopping distance rotations
            return Math.signum(currentError) * Math.sqrt(2.0 * maxAcelRotationsPerSecSquared * Math.abs(currentError));
        }
        
        // Accelerate toward max velocity
        double targetVel = Math.signum(currentError) * maxVelocityRotationsPerSec;
        
        // Make it so we don't aceed max acel
        double velocityDelta = targetVel - currentVelocityRotationsPerSec;
        double maxDelta = maxAcelRotationsPerSecSquared * 0.02;
        return currentVelocityRotationsPerSec + MathUtil.clamp(velocityDelta, -maxDelta, maxDelta);
        }

    @Override
    public void setTargetAimerPosition(double targetPositionDegreesRobotToTarget) {

        // testing counteracting the robot rotational velocity
        double robotRotationVelocityRotations = Units.radiansToRotations(drivetrain.getFieldOrientedVelocity().omegaRadiansPerSecond);
        double robotRotationFeedForward = -robotRotationVelocityRotations *kVVoltsVoltsPerRotation;
        // robotRotationFeedForward=robotRotationFeedForward;
     

        double feedForwardsSpringVolts = ksForConstantForceSpring;
        // double feedForwardsSpringAmps = ksForConstantForceSpringAmps;

        double turretPositionRotations = aimerKraken.getPosition().getValueAsDouble();
        double turretVelocityRotationsPerSec = aimerKraken.getVelocity().getValueAsDouble();
        // boolean inDeadZone = false;

        if(turretPositionRotations < 0) {
            feedForwardsSpringVolts = feedForwardsSpringVolts*-1.0;
        }

        double targetAngleDegreesTurretToTargetIfTurretWasFront = targetPositionDegreesRobotToTarget - drivetrain.getPoseMeters().getRotation().getDegrees();

        double targetAngleDeg180Clamped = Rotation2d.fromDegrees(targetAngleDegreesTurretToTargetIfTurretWasFront)
            .rotateBy(Rotation2d.kZero).getDegrees();

        double safeAngle = getSafeOptimizedAngleDeg(targetAngleDeg180Clamped);
        targetAimerDegrees = safeAngle;

        double pidOutputVolts;
        double errorRotations = (Units.degreesToRotations(safeAngle) -turretPositionRotations);

        double velocitySetpoint = motionProfile(errorRotations, turretVelocityRotationsPerSec, 2.0, 9.0);
        velocitySetpoint = velocitySetpoint -robotRotationVelocityRotations;

        double pidOutputToTarget;
        if(Units.rotationsToDegrees(errorRotations) < 15.0) {
            velocitySetpoint = -robotRotationVelocityRotations;
            turretPIDToTarget.setP(100.0);
            pidOutputToTarget = MathUtil.clamp(turretPIDToTarget.calculate(turretPositionRotations, Units.degreesToRotations(safeAngle)), -4.0,4.0);
        } else {
            turretPIDToTarget.setP(35.0);
            pidOutputToTarget = MathUtil.clamp(turretPIDToTarget.calculate(turretPositionRotations, Units.degreesToRotations(safeAngle)), -1.1, 1.1);
        }
        Logger.recordOutput("profile velcity setpoint", velocitySetpoint);
        Logger.recordOutput("aimer timer", timer.get());

        double pidOutputToVelocitySetpoint = MathUtil.clamp(turretPIDToSetpoint.calculate(turretVelocityRotationsPerSec, velocitySetpoint), -0.8, 0.8);

        pidOutputVolts = pidOutputToTarget + pidOutputToVelocitySetpoint;

        // double feedForwardVolts = feedForwardsSpringVolts + m_setpoint.velocity*kVVoltsVoltsPerRotation;
        double feedForwardVolts = feedForwardsSpringVolts + velocitySetpoint*kVVoltsVoltsPerRotation;
        // pidOutputVolts = turretPID.calculate(aimerKraken.getVelocity().getValueAsDouble(), profiledController.getSetpoint().velocity);
        double outputVolts = MathUtil.clamp(pidOutputVolts + feedForwardVolts, -8.0, 8.0);
        // double outputVolts = MathUtil.clamp(pidOutputVolts+ feedForwardsSpringVolts + robotRotationFeedForward, -8.0, 8.0);


        if(Math.abs(safeAngle-(Units.rotationsToDegrees(aimerKraken.getPosition().getValueAsDouble()))) > 80.0) {
            aimerKraken.setControl(motionMagic.withPosition(Units.degreesToRotations(safeAngle)).withFeedForward(feedForwardsSpringVolts + robotRotationFeedForward));
        } else {
            aimerKraken.setVoltage(outputVolts);
        }
    }

}