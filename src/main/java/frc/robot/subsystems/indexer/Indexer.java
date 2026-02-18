package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase{

    IndexerIOInputsAutoLogged indexerInputs;
    IndexerIO indexerIO;

    public Indexer(IndexerIO indexerIO) {
        this.indexerIO=indexerIO;
        indexerInputs = new IndexerIOInputsAutoLogged();
    }

    public void setAllTargetAmps(double kickerAmps, double sideKickerAmps, double bigSpinnerAmps) {
        indexerIO.setKickerAmps(kickerAmps);
        indexerIO.setSideKickerAmps(sideKickerAmps);
        indexerIO.setBigSpinnerAmps(bigSpinnerAmps);
    }

    public Command indexFuelCommand() {
        return(this.run(() -> new InstantCommand()));
    }

    
    public Command stopIndexingCommand() {
        return(this.run(() -> new InstantCommand()));
    }
}
