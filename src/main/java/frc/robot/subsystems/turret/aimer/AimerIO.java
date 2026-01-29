package frc.robot.subsystems.turret.aimer;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.Constants.TurretConstants;

public interface AimerIO {
    @AutoLog
    public class AimerIOInputs {
        public double aimerPositionDegrees = TurretConstants.aimerInitialPositionDegrees;
        public double aimerVelocityDegreesPerSecond = 0.0;

        public double aimerAppliedVoltage = 0.0;
        public double aimerAmps = 0.0;
    }

    public default void updateInputs(AimerIOInputs inputs) {};

    public default void setAimerVolts(double volts) {};

    public default void setTargetAimerPosition(double targetPositionDegrees) {};

}

