package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public class IndexerIOInputs {
        public double bigSpinnerVelocityRPS = 0.0;
        public double bigSpinnerTargetRPS = 0.0;
        public double bigSpinnerVolts = 0.0;
        public double bigSpinnerAmps = 0.0;

        public double sideKickerVelocityRPS = 0.0;
        public double sideKickerTargetRPS = 0.0;
        public double sideKickerVolts = 0.0;
        public double sideKickerAmps = 0.0;

        public double kickerVelocityRPS = 0.0;
        public double kickerTargetRPS = 0.0;
        public double kickerVolts = 0.0;
        public double kickerAmps = 0.0;
    }

    public default void updateInputs(IndexerIOInputs inputs) {};

    public default void setBigSpinnerVolts(double volts) {};

    public default void setSideKickerVolts(double volts) {};

    public default void setKickerVolts(double volts) {};

    public default void setBigSpinnerAmps(double volts) {};

    public default void setSideKickerAmps(double volts) {};

    public default void setKickerAmps(double volts) {};

    public default void setTargetBigSpinnerVelocity(double targetVelocityRPS) {};

    public default void setTargetSideKickerVelocity(double targetVelocityRPS) {};

    public default void setTargetKickerVelocity(double targetVelocityRPS) {};

}
