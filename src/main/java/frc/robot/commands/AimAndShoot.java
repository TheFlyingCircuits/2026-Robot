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
    private Supplier<TurretCalculations.possibeTargets>  shootingTarget;
    private Supplier<Boolean> driverReadyToShoot;
    private double angleOfAttack;
    private boolean isShooting = false;

    // 0 is aimer deg, 1 is hood deg, 2 is mainWheel M/S, 3 is hoodWheel M/S
    private double[] notShootingTolerances = new double[] {0.5, 0.5, 0.3, 0.3};
    private double[] whileShootingTolerances = new double[] {2.0, 2.0, 4.0, 4.0};

    

    public AimAndShoot(Turret turret, Indexer indexer, Supplier<Translation3d> turretTranlsation, Supplier<ChassisSpeeds> robotFieldOrientedVelocity,
    Supplier<TurretCalculations.possibeTargets> shootingTarget, Supplier<Boolean> driverReadyToShoot, boolean needsReq) {
        this.turret=turret;
        this.indexer=indexer;
        this.turretTranlsation=turretTranlsation;
        this.robotFieldOrientedVelocity=robotFieldOrientedVelocity;
        this.shootingTarget=shootingTarget;
        this.driverReadyToShoot=driverReadyToShoot;
        // this.angleOfAttack=angleOfAttack;
        isShooting = false;

        addRequirements(turret, indexer);

    }

    @Override
    public void initialize() {
        isShooting = false;
        TurretCalculations.currentTarget = shootingTarget.get();
    }

    @Override
    public void execute() {
        Translation3d originalTargetTranlsation = TurretCalculations.getTargetFromEnum(shootingTarget.get(), () -> turretTranlsation.get().toTranslation2d());

        angleOfAttack = TurretCalculations.getAngleOfAttackFromTargetEnum(shootingTarget.get(), (originalTargetTranlsation.toTranslation2d().minus(turretTranlsation.get().toTranslation2d())).getNorm());

        Translation3d targetMovmentCompensated = TurretCalculations.targetMovementCompensation(originalTargetTranlsation, 
            robotFieldOrientedVelocity.get(), angleOfAttack, turretTranlsation.get());

        // in the list 0 is output velocity, 1 is launch angle degrees, and 2 is time of impact seconds
        double[] shootingValues = TurretCalculations.angleOfAttackTrajCalc(targetMovmentCompensated, 
            angleOfAttack, turretTranlsation.get());

        // this is the angle that out robot would need to point to aim at the target
        double robotToTargetAngle = TurretCalculations.getAimerTargetDegreesRobotToTarget(targetMovmentCompensated.toTranslation2d(), turretTranlsation.get().toTranslation2d());

        // if we are shooting vs not shooting we have different tolerances
        boolean[] readyToShoot = isShooting ? turret.isReadyToShoot(whileShootingTolerances[0],whileShootingTolerances[1],whileShootingTolerances[2],whileShootingTolerances[3]) 
        : turret.isReadyToShoot(notShootingTolerances[0],whileShootingTolerances[1],whileShootingTolerances[2],whileShootingTolerances[3]);

        // driverReadyToShoot is a boolean based off driver button
        if(driverReadyToShoot.get()) {

            // if driver is ready to shoot we aim at the target with hood and aimer and rev flywheels
            turret.aimAtTargetAndShoot(robotToTargetAngle, shootingValues[1], shootingValues[0]);

            // if everything is ready to shoot in the turret subsystem we shoot by turning on indexer
            if(readyToShoot[0] && readyToShoot[1] && readyToShoot[2] && readyToShoot[3]) {
                indexer.indexFuelCommand();
                isShooting = true;
            } else {
                // stop indexer if turret is not ready to shoot or turret gets out of the tolerance while shooting
                indexer.stopIndexingCommand();
                isShooting = false;
            }
        } else {
            // aimer will just preaim target but hood will be at defualt position and flyWheels will be stationary
            turret.aimAtTargetNoShoot(robotToTargetAngle);
            indexer.stopIndexingCommand();
            isShooting = false;
        }

        // log values
        Logger.recordOutput("AimAndShoot/shooting", isShooting);
        Logger.recordOutput("AimAndShoot/driverReadyToShoot", driverReadyToShoot.get());
        Logger.recordOutput("AimAndShoot/aimerReady", readyToShoot[0]);
        Logger.recordOutput("AimAndShoot/hoodReady", readyToShoot[1]);
        Logger.recordOutput("AimAndShoot/mainWheel", readyToShoot[2]);
        Logger.recordOutput("AimAndShoot/hoodWheel", readyToShoot[3]);
        Logger.recordOutput("AimAndShoot/angleOfAttack", angleOfAttack);

        // call the log shooting calculations might get rid if causes performance issue I don't think it will though
        TurretCalculations.logShootingFunctions(originalTargetTranlsation, 
            robotFieldOrientedVelocity.get(), angleOfAttack, turretTranlsation.get());
        
    }
    
    
}
