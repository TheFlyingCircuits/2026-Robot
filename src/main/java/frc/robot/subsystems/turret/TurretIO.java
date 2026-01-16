package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public class TurretIOInputs {

    }

    public default void updateInputs(TurretIOInputs inputs) {};
}
