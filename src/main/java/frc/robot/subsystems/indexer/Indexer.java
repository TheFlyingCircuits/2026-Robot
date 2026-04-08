package frc.robot.subsystems.indexer;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
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
        Logger.processInputs("indexerInputs", indexerInputs);
    }

    public void setAllTargetAmps(double kickerAmps, double sideKickerAmps, double bigSpinnerAmps) {
        indexerIO.setKickerAmps(kickerAmps);
        indexerIO.setSideKickerAmps(sideKickerAmps);
        indexerIO.setBigSpinnerAmps(bigSpinnerAmps);
    }

    public void setAllTargetVolts(double kickerVolts, double sideKickerVolts, double bigSpinnerVolts) {
        indexerIO.setKickerVolts(kickerVolts);
        indexerIO.setSideKickerVolts(sideKickerVolts);
        indexerIO.setBigSpinnerVolts(bigSpinnerVolts);
    }

    public void setAllTargetVelocity(double kickerVelMPS, double sideKickerVelRPS, double bigSpinnerVelRPS) {
        double kickerVelRPS = kickerVelMPS
            /(Math.PI*Units.inchesToMeters(2.0));
        indexerIO.setTargetKickerVelocity(kickerVelRPS);
        indexerIO.setTargetSideKickerVelocity(sideKickerVelRPS);
        indexerIO.setTargetBigSpinnerVelocity(bigSpinnerVelRPS);
    }

    public void indexFuel() {
        setAllTargetVelocity(6.0,45.0,2.0);
    }
    
    public void shootFuel(double targetKickerMPS) {
        setAllTargetVelocity(targetKickerMPS*0.8,45.0,2.0);// 2.5 original
    }

    public void reverseIndexer() {
        setAllTargetVelocity(-5.0,-45.0,-2.5);
    }

    public void stopIndexing() {
        setAllTargetVolts(0.0,0.0,0.0);
    }

    public Command setAllTargetVelocityCommand(double kickerVelRPS, double sideKickerVelRPS, double bigSpinnerVelRPS) {
        return(this.run(() -> setAllTargetVelocity(kickerVelRPS, sideKickerVelRPS, bigSpinnerVelRPS)));
    }

    public Command kickerVolts(Supplier<Double> volts) {
        return this.run(() -> setAllTargetVolts(volts.get(), 0.0, 0.0));
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
