package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.TurretConstants;

public interface TurretIO {
    @AutoLog
    public class TurretIOInputs {
        public double aimerPositionDegrees = TurretConstants.aimerInitialPositionDegrees;
        public double hoodPositionDegrees = TurretConstants.hoodInitialPositionDegrees;
        
        public double aimerVelocityDegreesPerSecond = 0.0;
        public double hoodVelocityDegreesPerSecond = 0.0;

        public double frontWheelVelocityRPM = 0.0;
        public double hoodWheelVelocityRPM = 0.0;
    }

    public default void updateInputs(TurretIOInputs inputs) {};

    public default void setAimerVolts(double volts) {};

    public default void setFrontWheelVoltage(double volts) {};

    public default void setHoodWheelVolts(double volts) {};
    
    public default void setHoodVoltage(double volts) {};

    public default void setTargetAimerPosition(double targetPositionDegrees) {};

    public default void setTargetFrontWheelVelocity(double targetVelocityRMP) {};

    public default void setTargetHoodWheelVelocity(double targetVelocityRMP) {};
    
    public default void setTargetHoodPosition(double targetPositionDegrees) {};
}
