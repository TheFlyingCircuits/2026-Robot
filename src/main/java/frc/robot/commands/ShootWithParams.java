package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.turret.Turret;

public class ShootWithParams extends Command{

    private Turret turret;
    private Indexer indexer;
    private boolean isShooting = false;
    private Supplier<Double> aimerAngleDeg;
    private Supplier<Double> hoodAngleDeg;
    private Supplier<Double> targetShotMetersPerSecond;

    // 0 is aimer deg, 1 is hood deg, 2 is mainWheel M/S, 3 is hoodWheel M/S
    private double[] notShootingTolerances = new double[] {5.0, 5.0, 1.0, 1.0};
    private double[] whileShootingTolerances = new double[] {5.0, 5.0, 4.0, 4.0};

    

    public ShootWithParams(Turret turret, Indexer indexer, Supplier<Double> aimerAngleDeg, Supplier<Double> hoodAngleDeg,
        Supplier<Double> targetShotMetersPerSecond) {
        this.turret=turret;
        this.indexer=indexer;
        this.aimerAngleDeg=aimerAngleDeg;
        this.hoodAngleDeg=hoodAngleDeg;
        this.targetShotMetersPerSecond=targetShotMetersPerSecond;
        // this.angleOfAttack=angleOfAttack;
        isShooting = false;

        addRequirements(turret, indexer);
    }

    @Override
    public void initialize() {
        isShooting = false;
    }

    @Override
    public void execute() {

        // if we are shooting vs not shooting we have different tolerances
        Supplier<Boolean>[] readyToShoot = isShooting ? turret.isReadyToShoot(whileShootingTolerances[0],whileShootingTolerances[1],whileShootingTolerances[2],whileShootingTolerances[3]) 
        : turret.isReadyToShoot(notShootingTolerances[0],whileShootingTolerances[1],whileShootingTolerances[2],whileShootingTolerances[3]);

        // driverReadyToShoot is a boolean based off driver button

        // if driver is ready to shoot we aim at the target with hood and aimer and rev flywheels
        turret.aimAtTargetAndShoot(aimerAngleDeg.get(), hoodAngleDeg.get(), targetShotMetersPerSecond.get());

        // if everything is ready to shoot in the turret subsystem we shoot by turning on indexer
        if(readyToShoot[0].get().booleanValue() && readyToShoot[1].get().booleanValue() && readyToShoot[2].get().booleanValue() && readyToShoot[3].get().booleanValue()) {
            indexer.indexFuel();
            isShooting = true;
        } else {
            // stop indexer if turret is not ready to shoot or turret gets out of the tolerance while shooting
            indexer.stopIndexing();
            isShooting = false;
        }

        Logger.recordOutput("AimAndShoot/aimerReady", readyToShoot[0].get().booleanValue());
        Logger.recordOutput("AimAndShoot/hoodReady", readyToShoot[1].get().booleanValue());
        Logger.recordOutput("AimAndShoot/mainWheel", readyToShoot[2].get().booleanValue());
        Logger.recordOutput("AimAndShoot/hoodWheel", readyToShoot[3].get().booleanValue());
    } 
        
}
    
    

