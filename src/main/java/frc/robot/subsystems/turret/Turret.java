package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret {

    TurretIOInputsAutoLogged inputs;
    TurretIO io;

    Aimer aimer;
    Flywheels flywheels;
    Hood hood;

    public Turret(TurretIO io) {
        this.io = io;
        inputs = new TurretIOInputsAutoLogged();

        aimer = new Aimer();
        flywheels = new Flywheels();
        hood = new Hood();
    }

    public class Aimer extends SubsystemBase {

    }

    public class Flywheels extends SubsystemBase {

    }

    public class Hood extends SubsystemBase {

    }
    
}
