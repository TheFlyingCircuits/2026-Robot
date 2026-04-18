package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;

public class ShootWithParams extends Command {

    private Turret turret;
    private Indexer indexer;
    private Intake intake;
    private boolean isShooting = false;
    private Supplier<Double> aimerAngleDeg;
    private Supplier<Double> hoodAngleDeg;
    private Supplier<Double> targetShotMetersPerSecond;
    private Supplier<ChassisSpeeds> robotFieldOrientedVelocity;

    // 0 is aimer deg, 1 is hood deg, 2 is mainWheel M/S, 3 is hoodWheel M/S
    // private double[] notShootingTolerances = new double[] {5.0, 5.0, 1.0, 1.0};
    private double[] notShootingTolerances = new double[] {3.0, 1.3, 0.15};
    // private double[] whileShootingTolerances = new double[] {5.0, 5.0, 4.0, 4.0};
    private double[] whileShootingTolerances = new double[] {10.0, 5.0, 6.0};

    

    public ShootWithParams(Turret turret, Indexer indexer, Supplier<Double> aimerAngleDeg, Supplier<Double> hoodAngleDeg,
        Supplier<Double> targetShotMetersPerSecond, Intake intake, Supplier<ChassisSpeeds> robotFieldOrientedVelocity) {
        this.turret=turret;
        this.indexer=indexer;
        this.aimerAngleDeg=aimerAngleDeg;
        this.intake=intake;
        this.hoodAngleDeg=hoodAngleDeg;
        this.targetShotMetersPerSecond=targetShotMetersPerSecond;
        this.robotFieldOrientedVelocity=robotFieldOrientedVelocity;
        // this.angleOfAttack=angleOfAttack;
        isShooting = false;

        addRequirements(turret, indexer, intake);
    }

    @Override
    public void initialize() {
        isShooting = false;
    }

    @Override
    public void execute() {
        if(Math.abs(robotFieldOrientedVelocity.get().vxMetersPerSecond) + Math.abs(robotFieldOrientedVelocity.get().vyMetersPerSecond) < 0.025) {
            // intake.intakeUpDown();
            intake.intakeDefualtAndIntake();
        } else {
            intake.intakeDefualtAndIntake();
        }
        // intake.intakeUpDown();

        // driverReadyToShoot is a boolean based off driver button
        // double wheelVelocityTarget = TurretConstants.velocityLookUp.get(targetShotMetersPerSecond.get());
        double wheelVelocityTarget = targetShotMetersPerSecond.get();

        // if driver is ready to shoot we aim at the target with hood and aimer and rev flywheels
        turret.aimAtTargetAndShoot(aimerAngleDeg.get(), hoodAngleDeg.get(), wheelVelocityTarget);

        // if we are shooting vs not shooting we have different tolerances
        Supplier<Boolean>[] readyToShoot = isShooting ? turret.isReadyToShoot(whileShootingTolerances[0],whileShootingTolerances[1],whileShootingTolerances[2]) 
        : turret.isReadyToShoot(notShootingTolerances[0],notShootingTolerances[1],notShootingTolerances[2]);

        // if everything is ready to shoot in the turret subsystem we shoot by turning on indexer
        if(readyToShoot[0].get().booleanValue() && readyToShoot[1].get().booleanValue() && readyToShoot[2].get().booleanValue()) {
            indexer.shootFuel(targetShotMetersPerSecond.get());
            isShooting = true;
        } else {
            // stop indexer if turret is not ready to shoot or turret gets out of the tolerance while shooting
            indexer.stopIndexing();
            isShooting = false;
        }

        Logger.recordOutput("AimAndShoot/aimerReady", readyToShoot[0].get().booleanValue());
        Logger.recordOutput("AimAndShoot/hoodReady", readyToShoot[1].get().booleanValue());
        Logger.recordOutput("AimAndShoot/mainWheel", readyToShoot[2].get().booleanValue());
        // Logger.recordOutput("AimAndShoot/hoodWheel", readyToShoot[3].get().booleanValue());
    } 
        
}
    
    

