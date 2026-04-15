package frc.robot.subsystems.indexer;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;

public class Indexer extends SubsystemBase {

    IndexerIOInputsAutoLogged indexerInputs;
    IndexerIO indexerIO;
    IndexerIOInputs indexerIOInputs;
    LinearFilter filter;
    Timer timer;

    public Indexer(IndexerIO indexerIO) {
        this.indexerIO=indexerIO;
        indexerInputs = new IndexerIOInputsAutoLogged();
        indexerIOInputs = new IndexerIOInputs();
        filter = LinearFilter.movingAverage(10);
        timer = new Timer();
    }

    @Override
    public void periodic() {
        indexerIO.updateInputs(indexerInputs);
        Logger.processInputs("indexerInputs", indexerInputs);
        filter.calculate(indexerIOInputs.bigSpinnerAmps);
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
        setAllTargetVelocity(4.5,45.0,2.5);
    }
    
    public void shootFuel(double targetKickerMPS) {
        if (filter.lastValue() > 145) 
            timer.start();
        if (timer.get() > 0) 
            setAllTargetVelocity(4.5,45.0,-2.5);// 2.5 original
            if (timer.get() > .2) {
                timer.stop();
                timer.reset();
            }
        else 
            setAllTargetVelocity(4.5,45.0,2.5);// 2.5 original

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
