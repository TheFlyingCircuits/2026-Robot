package frc.robot.subsystems.turret.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.UniversalConstants;
import frc.robot.VendorWrappers.Kraken;

public class HoodIOKraken implements HoodIO{
    private Kraken hoodKraken;
    private double targetHoodDegreesLocal = 0.0;

    private double hoodFeedForwardGravity = -0.7;

    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0).withEnableFOC(true)
        .withUpdateFreqHz(60.0);

    private final PositionTorqueCurrentFOC positionTorqueFOC = new PositionTorqueCurrentFOC(0.0).withSlot(1)
    .withUpdateFreqHz(0.0);

    public HoodIOKraken() {
        hoodKraken = new Kraken(TurretConstants.hoodKrakenID, UniversalConstants.canivoreName);
        configHoodKraken();
    }

    private void configHoodKraken() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 85;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        // replaced kS with a constant volts feed forward for gravity because ks would switch if going down
        // to negative volts and I don't want that
        config.Slot0.kS = 0.11; // Add 0.6 V output to overcome static friction
        config.Slot0.kP = 162.5; // An error of 1 rotation results in 20.0 V output for each 1/8th rot off 2.5V// was 154
        config.Slot0.kI = 0.0; 
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.0;

        config.Slot0.kS = 0.1;
        config.Slot1.kP = 0.0;
        config.Slot1.kD = 0.0;


        config.ClosedLoopGeneral.GainSchedErrorThreshold = Units.degreesToRotations(0.0);
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = TurretConstants.hoodKrakenToTurretRotationsGearRatio;
        config.ClosedLoopGeneral.ContinuousWrap = false;
        hoodKraken.applyConfig(config);
        hoodKraken.setPosition(Units.degreesToRotations(TurretConstants.maxHoodAngle));
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodPositionDegrees = Units.rotationsToDegrees(hoodKraken.getPosition().getValueAsDouble());
        inputs.hoodVelocityDegreesPerSecond = Units.rotationsToDegrees(hoodKraken.getVelocity().getValueAsDouble());
        inputs.targetHoodPositionDegrees = targetHoodDegreesLocal;

        inputs.hoodAppliedVoltage = hoodKraken.getMotorVoltage().getValueAsDouble();
        inputs.hoodAmps = hoodKraken.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setHoodVolts(double volts) {
        hoodKraken.setVoltage(volts);
    }

    @Override
    public void setHoodAmps(double amps) {
        hoodKraken.setControl(new TorqueCurrentFOC(amps));
    }

    @Override
    public void setTargetHoodPosition(double targetPositionDegrees) {
        targetHoodDegreesLocal = targetPositionDegrees;
        if(targetPositionDegrees < TurretConstants.minHoodAngle + 0.5) {
            targetPositionDegrees = TurretConstants.minHoodAngle + 0.5;
        } else if(targetPositionDegrees > TurretConstants.maxHoodAngle -0.5) {
            targetPositionDegrees = TurretConstants.maxHoodAngle - 0.5;
        }
        hoodKraken.setControl(m_request.withPosition(Units.degreesToRotations(targetPositionDegrees)).withFeedForward(hoodFeedForwardGravity));
        // hoodKraken.setControl(positionTorqueFOC.withPosition(Units.degreesToRotations(targetPositionDegrees)).withFeedForward(hoodFeedForwardGravity));
    }

}