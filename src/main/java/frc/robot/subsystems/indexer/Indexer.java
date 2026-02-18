package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
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

    public void setAllTargetVolts(double kickerVolts, double sideKickerVolts, double bigSpinnerVolts) {
        indexerIO.setKickerAmps(kickerVolts);
        indexerIO.setSideKickerAmps(sideKickerVolts);
        indexerIO.setBigSpinnerAmps(bigSpinnerVolts);
    }

    public void setAllTargetVelocity(double kickerVelRPS, double sideKickerVelRPS, double bigSpinnerVelRPS) {
        indexerIO.setTargetKickerVelocity(kickerVelRPS);
        indexerIO.setTargetSideKickerVelocity(sideKickerVelRPS);
        indexerIO.setTargetBigSpinnerVelocity(bigSpinnerVelRPS);
    }

    public Command setAllTargetVelocityCommand(double kickerVelRPS, double sideKickerVelRPS, double bigSpinnerVelRPS) {
        return(this.run(() -> setAllTargetVelocity(kickerVelRPS, sideKickerVelRPS, bigSpinnerVelRPS)));
    }


    // EDIT THIS FOR DIFFERENT INDEXER SPEEDS
    public Command indexFuelCommand() {
        return(this.run(() -> setAllTargetVelocity(30.0,30.0,2.0)));
    }

    
    public Command stopIndexingCommand() {
        return(this.run(() -> setAllTargetVelocity(0.0,0.0,0.0)));
    }
}
