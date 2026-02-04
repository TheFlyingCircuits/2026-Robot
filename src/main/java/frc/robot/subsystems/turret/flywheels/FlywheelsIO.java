package frc.robot.subsystems.turret.flywheels;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.TurretConstants;

public interface FlywheelsIO {
    @AutoLog
    public class FlywheelsIOInputs {
        public double aimerPositionDegrees = TurretConstants.aimerInitialPositionDegrees;
        public double hoodPositionDegrees = TurretConstants.hoodInitialPositionDegrees;
        
        public double aimerVelocityDegreesPerSecond = 0.0;
        public double hoodVelocityDegreesPerSecond = 0.0;

        public double frontWheelVelocityRPS = 0.0;
        public double hoodWheelVelocityRPS = 0.0;

        public double frontWheelVelocityMPS = 0.0;
        public double hoodWheelVelocityMPS = 0.0;

        public double targetFrontWheelVelocityMPS = 0.0;
        public double targetHoodWheelVelocityMPS = 0.0;

        public double frontWheelAppliedVoltage = 0.0;
        public double frontWheelFollowerAppliedVoltage = 0.0;
        public double hoodWheelAppledVoltage = 0.0;

        public double frontWheelAmps = 0.0;
        public double frontWheelFollowerAmps= 0.0;
        public double hoodWheelAmps = 0.0;
    }

    public default void updateInputs(FlywheelsIOInputs inputs) {};

    public default void setFrontWheelVolts(double volts) {};

    public default void setFrontFollowerWheelVolts(double volts) {};

    public default void setHoodWheelVolts(double volts) {};

    public default void setTargetFrontWheelVelocity(double targetVelocityRPS) {};

    public default void setTargetHoodWheelVelocity(double targetVelocityRPS) {};

}
