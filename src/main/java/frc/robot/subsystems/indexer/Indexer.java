package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

    IndexerIOInputsAutoLogged indexerInputs;
    IndexerIO indexerIO;

    public Indexer(IndexerIO indexerIO) {
        this.indexerIO=indexerIO;
        indexerInputs = new IndexerIOInputsAutoLogged();
    }

    @Override
    public void periodic() {
        indexerIO.updateInputs(indexerInputs);
        Logger.processInputs("aimerInputs", indexerInputs);
    }

    public void setAllTargetAmps(double kickerAmps, double sideKickerAmps, double bigSpinnerAmps) {
        indexerIO.setKickerAmps(kickerAmps);
        indexerIO.setSideKickerAmps(sideKickerAmps);
        indexerIO.setBigSpinnerAmps(bigSpinnerAmps);
    }

    public void setAllTargetVolts(double kickerVolts, double sideKickerVolts, double bigSpinnerVolts) {
        indexerIO.setBigSpinnerVolts(kickerVolts);
        indexerIO.setBigSpinnerVolts(sideKickerVolts);
        indexerIO.setBigSpinnerVolts(bigSpinnerVolts);
    }

    public void setAllTargetVelocity(double kickerVelRPS, double sideKickerVelRPS, double bigSpinnerVelRPS) {
        indexerIO.setTargetKickerVelocity(kickerVelRPS);
        indexerIO.setTargetSideKickerVelocity(sideKickerVelRPS);
        indexerIO.setTargetBigSpinnerVelocity(bigSpinnerVelRPS);
    }

    public void indexFuel() {
        setAllTargetVelocity(50.0,50.0,2.5);
    }

    public void reverseIndexer() {
        setAllTargetVelocity(-50.0,-50.0,-2.5);
    }

    public void stopIndexing() {
        setAllTargetVolts(0.0,0.0,0.0);
    }

    public Command setAllTargetVelocityCommand(double kickerVelRPS, double sideKickerVelRPS, double bigSpinnerVelRPS) {
        return(this.run(() -> setAllTargetVelocity(kickerVelRPS, sideKickerVelRPS, bigSpinnerVelRPS)));
    }


    // EDIT THIS FOR DIFFERENT INDEXER SPEEDS
    public Command indexFuelCommand() {
        return(this.run(() -> indexFuel()));
    }

    public Command reverseIndexerCommand() {
        return(this.run(() -> reverseIndexer()));
    }

    public Command stopIndexingCommand() {
        return(this.run(() -> stopIndexing()));
    }
}
