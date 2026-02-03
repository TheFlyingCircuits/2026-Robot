package frc.robot.commands;

import java.util.function.Supplier;

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

        if(driverReadyToShoot.get()) {
            turret.aimAtTargetAndShootCommand(()->robotToTargetAngle, ()->shootingValues[1], ()->shootingValues[0]);
            indexer.indexFuelCommand();
        } else {
            turret.aimAtTargetNoShootCommand(()->robotToTargetAngle);
            indexer.stopIndexingCommand();
        }

        
    }
    
    
}
