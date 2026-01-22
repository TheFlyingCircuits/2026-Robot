package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{

    AimerIOInputsAutoLogged aimerInputs;
    FlywheelsIOInputsAutoLogged flywheelsInputs;
    HoodIOInputsAutoLogged hoodInputs;

    AimerIO aimer;
    FlywheelsIO flywheels;
    HoodIO hood;

    public Turret(AimerIO aimer, FlywheelsIO flywheels, HoodIO hood) {
        this.aimer = aimer;
        this.flywheels = flywheels;
        this.hood = hood;

        aimerInputs = new AimerIOInputsAutoLogged();
        flywheelsInputs = new FlywheelsIOInputsAutoLogged();
        hoodInputs = new HoodIOInputsAutoLogged();
    }
    
}
