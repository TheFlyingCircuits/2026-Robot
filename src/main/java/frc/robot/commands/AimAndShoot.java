package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretCalculations;

public class AimAndShoot extends Command{

    private Turret turret;
    private Indexer indexer;
    private Supplier<Translation3d> turretTranlsation;
    private Supplier<ChassisSpeeds> robotFieldOrientedVelocity;
    private Supplier<Translation3d> shootingTarget;
    private Supplier<Boolean> driverReadyToShoot;
    private Supplier<Double> angleOfAttack;

    

    public AimAndShoot(Turret turret, Indexer indexer, Supplier<Translation3d> turretTranlsation, Supplier<ChassisSpeeds> robotFieldOrientedVelocity,
    Supplier<Translation3d> shootingTarget, Supplier<Boolean> driverReadyToShoot, Supplier<Double> angleOfAttack) {
        this.turret=turret;
        this.indexer=indexer;
        this.turretTranlsation=turretTranlsation;
        this.robotFieldOrientedVelocity=robotFieldOrientedVelocity;
        this.shootingTarget=shootingTarget;
        this.driverReadyToShoot=driverReadyToShoot;
        this.angleOfAttack=angleOfAttack;

        addRequirements(turret, indexer);
    }

    @Override
    public void execute() {
        Translation3d targetMovmentCompensated = TurretCalculations.targetMovementCompensation(shootingTarget.get(), 
            robotFieldOrientedVelocity.get(), angleOfAttack.get(), turretTranlsation.get());

        // in the list 0 is output velocity, 1 is launch angle degrees, and 2 is time of impact seconds
        double[] shootingValues = TurretCalculations.angleOfAttackTrajCalc(targetMovmentCompensated, 
            angleOfAttack.get(), turretTranlsation.get());

        double robotToTargetAngle = TurretCalculations.getAimerTargetDegrees(targetMovmentCompensated.toTranslation2d(), turretTranlsation.get().toTranslation2d());

        boolean[] readyToShoot = turret.isReadyToShoot(1.0,0.5,0.4,0.4);

        if(driverReadyToShoot.get()) {

            turret.aimAtTargetAndShoot(robotToTargetAngle, shootingValues[1], shootingValues[0]);

            if(readyToShoot[0] && readyToShoot[1] && readyToShoot[2] && readyToShoot[3]) {
                indexer.indexFuelCommand();
                Logger.recordOutput("AimAndShoot/shooting", true);
            } else {
                indexer.stopIndexingCommand();
                Logger.recordOutput("AimAndShoot/shooting", false);
            }
        } else {
            turret.aimAtTargetNoShoot(robotToTargetAngle);
            indexer.stopIndexingCommand();
            Logger.recordOutput("AimAndShoot/shooting", false);
        }

        Logger.recordOutput("AimAndShoot/driverReadyToShoot", driverReadyToShoot.get());
        Logger.recordOutput("AimAndShoot/aimerReady", readyToShoot[0]);
        Logger.recordOutput("AimAndShoot/hoodReady", readyToShoot[1]);
        Logger.recordOutput("AimAndShoot/mainWheel", readyToShoot[2]);
        Logger.recordOutput("AimAndShoot/hoodWheel", readyToShoot[3]);
        TurretCalculations.logShootingFunctions(shootingTarget.get(), 
            robotFieldOrientedVelocity.get(), angleOfAttack.get(), turretTranlsation.get());
        
    }
    
    
}
