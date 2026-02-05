package frc.robot.subsystems.turret.aimer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.TurretConstants;
import frc.robot.VendorWrappers.Kraken;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AimerIOKraken implements AimerIO{
    
    private Kraken aimerKraken;
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(1);
    private final VelocityVoltage m_Velrequest = new VelocityVoltage(0).withSlot(0);

    // DO NOT use anything besides get pose meters we don't want conflicts
    private Drivetrain drivetrain;

    private double targetAimerDegrees = 0.0;


    public AimerIOKraken(Drivetrain drivetrain) {
        // aimerKraken = new Kraken(TurretConstants.aimerKrakenID, UniversalConstants.canivoreName);
        aimerKraken = new Kraken(TurretConstants.aimerKrakenID, "rio");
        configAimerKraken();
        this.drivetrain=drivetrain;
    }

    private void configAimerKraken() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 25;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kS = 0.22; // Add 0.1 V output to overcome static friction
        config.Slot0.kP = 5.0; // An error of 1 rotation results in 0.5 V output
        config.Slot0.kI = 0.0; 
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.285;

        config.Slot1.kS = 0.22; // Add 0.1 V output to overcome static friction
        config.Slot1.kP = 60.0; // An error of 1 rotation results in 0.5 V output

        config.MotionMagic.MotionMagicCruiseVelocity = 1000.0; //rps of the arm
        config.MotionMagic.MotionMagicAcceleration = 200.0; //rotations per second squared of the arm
        // Units.degreesToRotations(1100);
        config.ClosedLoopGeneral.GainSchedErrorThreshold = Units.degreesToRotations(0.3);
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = TurretConstants.turretKrakenToTurretRotationsGearRatio;
        config.ClosedLoopGeneral.ContinuousWrap = false;
        aimerKraken.applyConfig(config);
        aimerKraken.setPosition(0.0);
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
            aimerKraken.setControl(new MotionMagicVoltage(Units.degreesToRotations(targetAngleDegreesTurretToTarget)));
        } else {
            aimerKraken.setControl(m_request.withPosition(Units.degreesToRotations(targetAngleDegreesTurretToTarget)));
        }
        // aimerKraken.setControl(m_Velrequest.withVelocity(Units.degreesToRotations(targetPositionDegrees)));
    }

}
