package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{
    

    public Command indexFuelCommand() {
        return(this.run(() -> new InstantCommand()));
    }

    
    public Command stopIndexingCommand() {
        return(this.run(() -> new InstantCommand()));
    }
}
