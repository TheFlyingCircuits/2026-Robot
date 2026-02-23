package frc.robot.subsystems.turret.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    @AutoLog
    public class HoodIOInputs {
        public double hoodPositionDegrees = 0.0;
        public double hoodVelocityDegreesPerSecond = 0.0;
        public double targetHoodPositionDegrees;

        public double hoodAppliedVoltage = 0.0;
        public double hoodAmps = 0.0;
    }

    public default void updateInputs(HoodIOInputs inputs) {};

    public default void setHoodVolts(double volts) {};

    public default void setTargetHoodPosition(double targetPositionDegrees) {};
}
