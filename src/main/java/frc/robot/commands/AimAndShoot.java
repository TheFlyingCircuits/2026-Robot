package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AdvantageScopeDrawingUtils;
import frc.robot.Constants.TurretConstants;
import frc.robot.FlyingCircuitUtils;
import frc.robot.PlayingField.FieldElement;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretCalculations;
import frc.robot.subsystems.turret.TurretCalculations.PossibeTargets;

public class AimAndShoot extends Command {

    private Turret turret;
    private Indexer indexer;
    private Intake intake;
    private Supplier<Translation3d> turretTranslation;
    private Supplier<ChassisSpeeds> robotFieldOrientedVelocity;
    private Supplier<TurretCalculations.PossibeTargets>  shootingTarget;
    private Supplier<Boolean> driverReadyToShoot;
    private Supplier<Boolean> shouldIntake;
    private double angleOfAttack;
    private boolean isShooting = false;
    private Drivetrain drivetrain;

    Translation3d originalTargetTranlsation;
    double[] shootingValues;
    Translation3d fuelNotCompensatedVelocity;
    Translation3d fuelVelocityCompensated;

    // 0 is aimer deg, 1 is hood deg, 2 is mainWheel M/S, 3 is hoodWheel M/S
    private double[] notShootingTolerances = new double[] {2.7, 1.5, 0.25, 0.25};
    private double[] whileShootingTolerances = new double[] {25.0, 15.0, 8.0, 8.0};

    

    public AimAndShoot(Turret turret, Indexer indexer, Supplier<Translation3d> turretTranlsation, Supplier<ChassisSpeeds> robotFieldOrientedVelocity,
    Supplier<Boolean> driverReadyToShoot, boolean needsReq, Drivetrain drivetrain, Intake intake, Supplier<Boolean> shouldIntake) {
        this.turret=turret;
        this.indexer=indexer;
        this.intake=intake;
        this.turretTranslation=turretTranlsation;
        this.robotFieldOrientedVelocity=robotFieldOrientedVelocity;
        this.driverReadyToShoot=driverReadyToShoot;
        this.shouldIntake=shouldIntake;
        this.drivetrain=drivetrain;
        isShooting = false;

        // drivetrain.allowTeleportsNextPoseUpdate();
        // drivetrain.fullyTrustVisionNextPoseUpdate();

        addRequirements(turret, indexer, intake);
    }

    @Override
    public void initialize() {
        isShooting = false;
        drivetrain.allowTeleportsNextPoseUpdate();
        drivetrain.fullyTrustVisionNextPoseUpdate();
    }

    private double getConvertedVelocity(double requestedOutputVelocityMPS) {
        // the velocity conversion factor (VCF)
        // https://www.desmos.com/calculator/vjiv7dbdtf

        // our max surface speed on our hood is about 14.8 so if around max speed cap it
        //23.8325407491 is out max front mps
        // if(requestedOutputVelocityMPS > 14.25) return 14.25;
        //y=0.0917253x^{2}+0.54115x+0.00240854
        // double wheelVelocityTarget = 0.0917253*Math.pow(requestedOutputVelocityMPS, 2)+0.54115*requestedOutputVelocityMPS+0.00240854;
        double wheelVelocityTarget = TurretConstants.velocityLookUp.get(requestedOutputVelocityMPS);
        // double wheelVelocityTarget = requestedOutputVelocityMPS + FlyingCircuitUtils.getNumberFromDashboard("MPS to add", 0.0);
        // double wheelVelocityTarget = requestedOutputVelocityMPS * 1.91 + -6.0;
            // FlyingCircuitUtils.getNumberFromDashboard("proportion", 1.91) +
            // FlyingCircuitUtils.getNumberFromDashboard("intercept", -6.0);

        // Logger.recordOutput("AimAndShoot/fudge_proportion", FlyingCircuitUtils.getNumberFromDashboard("proportion", 1.91));
        // Logger.recordOutput("AimAndShoot/fudge_intercept", FlyingCircuitUtils.getNumberFromDashboard("intercept", -6.0));

        // if VCF output is lower than original request return original velocity
        // if(requestedOutputVelocityMPS > wheelVelocityTarget) return requestedOutputVelocityMPS;

        return wheelVelocityTarget;
    }

    private void updateShootingTarget() {

        // field origin is always blue
        // Default to shooting at hub if we can't
        // tell what alliance we're on for whatever reason.
        boolean shouldTargetHub_blue = turretTranslation.get().getX() < FieldElement.HUB.getLocation().getX();
        boolean shouldTargetHub_red = !shouldTargetHub_blue;
        boolean shouldTargetHub = FlyingCircuitUtils.getAllianceDependentValue(shouldTargetHub_red, shouldTargetHub_blue, true);

        if (shouldTargetHub) {
            shootingTarget = () -> PossibeTargets.HUB;
            if(driverReadyToShoot.get()) {
                drivetrain.setFocus(FieldElement.HUB);
            } else {
                drivetrain.resetFocus();
            }
        } else {

            drivetrain.resetFocus();
            double sideWaysToleranceMeters = 0.75;

            double distanceToLeftTrench = FieldElement.TRENCH_LEFT.getLocation2d().getDistance(turretTranslation.get().toTranslation2d());
            double distanceToRightTrench = FieldElement.TRENCH_RIGHT.getLocation2d().getDistance(turretTranslation.get().toTranslation2d());

            try {
                if(shootingTarget.get() == PossibeTargets.PASSING_LEFT || shootingTarget.get() == PossibeTargets.PASSING_MID_LEFT) {
                    if(!(shootingTarget.get() == PossibeTargets.PASSING_MID_LEFT)) {
                        distanceToLeftTrench = distanceToLeftTrench - sideWaysToleranceMeters;
                    }
                    distanceToLeftTrench = distanceToLeftTrench - sideWaysToleranceMeters;
                } else {
                    if(!(shootingTarget.get() == PossibeTargets.PASSING_MID_RIGHT)) {
                        distanceToRightTrench = distanceToRightTrench - sideWaysToleranceMeters;
                    }
                }
                
                
            } catch(NullPointerException e) {}


            boolean isPastRedHub = turretTranslation.get().getX() > FieldElement.Enemy_HUB.getLocation().getX();
            boolean isPastBlueHub = !isPastRedHub;
            boolean shouldMidPass = FlyingCircuitUtils.getAllianceDependentValue(isPastBlueHub, isPastRedHub, true);
            if(distanceToLeftTrench<distanceToRightTrench) {
                if(!(shouldMidPass)) {
                    shootingTarget = ()-> PossibeTargets.PASSING_LEFT;
                } else {
                    shootingTarget = ()-> PossibeTargets.PASSING_MID_LEFT;
                }
            } else {
                if(!(shouldMidPass)) {
                    shootingTarget = ()-> PossibeTargets.PASSING_RIGHT;
                } else {
                    shootingTarget = ()-> PossibeTargets.PASSING_MID_RIGHT;
                }
            }
        }
    }

    @Override
    public void execute() {
        this.updateShootingTarget();


        originalTargetTranlsation = TurretCalculations.getTargetFromEnum(shootingTarget.get(), () -> turretTranslation.get().toTranslation2d());

        angleOfAttack = TurretCalculations.getAngleOfAttackFromTargetEnum(shootingTarget.get(), (originalTargetTranlsation.toTranslation2d().minus(turretTranslation.get().toTranslation2d())).getNorm());

        // Translation3d targetMovmentCompensated = TurretCalculations.targetMovementCompensation(originalTargetTranlsation, 
        //     robotFieldOrientedVelocity.get(), angleOfAttack, turretTranslation.get());

        // in the list 0 is output velocity in mps, 1 is launch angle degrees, and 2 is time of impact seconds
        shootingValues = TurretCalculations.angleOfAttackTrajCalc(originalTargetTranlsation, 
            angleOfAttack, turretTranslation.get());

        double xyVelocityComponent = shootingValues[0] * Math.cos(Units.degreesToRadians(shootingValues[1]));
        double zVelocityComponent = shootingValues[0] * Math.sin(Units.degreesToRadians(shootingValues[1]));
        double aimerTargetDegreesNotCompensated = TurretCalculations.getAimerTargetDegreesRobotToTarget(originalTargetTranlsation.toTranslation2d(), 
            turretTranslation.get().toTranslation2d());


        fuelNotCompensatedVelocity = new Translation3d(xyVelocityComponent*Math.cos(Units.degreesToRadians(aimerTargetDegreesNotCompensated)),
             xyVelocityComponent*Math.sin(Units.degreesToRadians(aimerTargetDegreesNotCompensated)), zVelocityComponent);

        fuelVelocityCompensated = fuelNotCompensatedVelocity.minus(new Translation3d(robotFieldOrientedVelocity.get().vxMetersPerSecond, 
            robotFieldOrientedVelocity.get().vyMetersPerSecond, 0));

        double vcfWheelMPS = getConvertedVelocity(fuelVelocityCompensated.getNorm());
        double targetAimerDed = Units.radiansToDegrees(Math.atan2(fuelVelocityCompensated.getY(), fuelVelocityCompensated.getX()));
        double tagetHoodAngle = Units.radiansToDegrees(Math.atan2(fuelVelocityCompensated.getZ(), fuelVelocityCompensated.toTranslation2d().getNorm()));

        // this is the angle that out robot would need to point to aim at the target
        // double robotToTargetAngle = TurretCalculations.getAimerTargetDegreesRobotToTarget(originalTargetTranlsation.toTranslation2d(), turretTranslation.get().toTranslation2d());

        // if we are shooting vs not shooting we have different tolerances
        Supplier<Boolean>[] readyToShoot;

        // driverReadyToShoot is a boolean based off driver button
        if(driverReadyToShoot.get()) {

            if(Math.abs(robotFieldOrientedVelocity.get().vxMetersPerSecond) + Math.abs(robotFieldOrientedVelocity.get().vyMetersPerSecond) < 0.025) {
                // intake.intakeUpDown();
                intake.intakeDefualtAndIntake();
            } else {
                intake.intakeDefualtAndIntake();
            }
            // if driver is ready to shoot we aim at the target with hood and aimer and rev flywheels
            
            // caps both speeds to almost max
            // double hoodMPSReq = (vcfWheelMPS > 14.25) ? 14.25 : vcfWheelMPS;
            double mainMPSReq = (vcfWheelMPS > 23.0) ? 23.0 : vcfWheelMPS;
            turret.aimAtTargetAndShoot(targetAimerDed, tagetHoodAngle, mainMPSReq); // * 1.65 too much *1.61)-1.385 woked good
            //*1.58)-1.37 good middle bad from far *1.58)-1.37 was at fingerlakes // *1.585)-1.385 overshot *1.58)-1.37

            readyToShoot = isShooting ? turret.isReadyToShoot(whileShootingTolerances[0],whileShootingTolerances[1],whileShootingTolerances[2]) 
            : turret.isReadyToShoot(notShootingTolerances[0],notShootingTolerances[1],notShootingTolerances[2]);

            // if everything is ready to shoot in the turret subsystem we shoot by turning on indexer
            if(readyToShoot[0].get().booleanValue() && readyToShoot[1].get().booleanValue() && readyToShoot[2].get().booleanValue()) {
                // indexer.shootFuel(shootingValues[0]*0.7);
                indexer.shootFuel(vcfWheelMPS);
                isShooting = true;
            } else {
                // stop indexer if turret is not ready to shoot or turret gets out of the tolerance while shooting
                indexer.stopIndexing();
                isShooting = false;
            }
        } else {
            if(shouldIntake.get()) {
                intake.intakeDefualtAndIntake();
            } else {
                intake.intakeRollersStop();
            }
            // aimer will just preaim target but hood will be at defualt position and flyWheels will be stationary
            turret.aimAtTargetNoShoot(targetAimerDed);

            readyToShoot = isShooting ? turret.isReadyToShoot(whileShootingTolerances[0],whileShootingTolerances[1],whileShootingTolerances[2]) 
            : turret.isReadyToShoot(notShootingTolerances[0],notShootingTolerances[1],notShootingTolerances[2]);

            indexer.stopIndexing();
            isShooting = false;
        }

        // log values
        // Logger.recordOutput("AimAndShoot/shooting", isShooting);
        Logger.recordOutput("AimAndShoot/driverReadyToShoot", driverReadyToShoot.get());
        Logger.recordOutput("AimAndShoot/aimerReady", readyToShoot[0].get().booleanValue());
        Logger.recordOutput("AimAndShoot/hoodReady", readyToShoot[1].get().booleanValue());
        Logger.recordOutput("AimAndShoot/mainWheel", readyToShoot[2].get().booleanValue());
        // Logger.recordOutput("AimAndShoot/hoodWheel", readyToShoot[3].get().booleanValue());
        // Logger.recordOutput("AimAndShoot/angleOfAttack", angleOfAttack);

        // call the log shooting calculations might get rid if causes performance issue I don't think it will though
        TurretCalculations.logShootingFunctions(originalTargetTranlsation, 
            robotFieldOrientedVelocity.get(), angleOfAttack, turretTranslation.get());
        // this.logPredictedTrajectory();
        Logger.recordOutput("AimAndShoot/desiredSpeedWithoutFudge", fuelVelocityCompensated.getNorm());
        Logger.recordOutput("AimAndShoot/desiredSpeedWithVCF", vcfWheelMPS);
        
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.resetFocus();
        AdvantageScopeDrawingUtils.eraseDrawing("predictedShot/withoutRobotVelocity");
        AdvantageScopeDrawingUtils.eraseDrawing("predictedShot/withRobotVelocity");
    }

    private void logPredictedTrajectory() {
        Translation3d initialPosition = turretTranslation.get();

        Rotation2d turretYaw_robotCoords = Rotation2d.fromDegrees(turret.getAimerAngleDeg_robotCoords());
        Rotation2d turretYaw_fieldCoords = turretYaw_robotCoords.plus(drivetrain.getPoseMeters().getRotation());
        
        // wpilib pitch convention is: positive angles <-> looking down.
        double turretPitchRadians = -Units.degreesToRadians(turret.getHoodAngleDeg());

        Rotation3d turretOrientation = new Rotation3d(0, turretPitchRadians, turretYaw_fieldCoords.getRadians());

        Translation3d estimatedExitVelocity = new Translation3d(turret.getAvgFlywheelSurfaceSpeedMetersPerSecond(), 0, 0).rotateBy(turretOrientation);

        AdvantageScopeDrawingUtils.drawProjectileMotion("predictedShot/withoutRobotVelocity", initialPosition, estimatedExitVelocity);
        
        Translation3d robotVelocity = new Translation3d(drivetrain.getFieldOrientedVelocity().vxMetersPerSecond, drivetrain.getFieldOrientedVelocity().vyMetersPerSecond, 0);
        estimatedExitVelocity = estimatedExitVelocity.plus(robotVelocity);
        AdvantageScopeDrawingUtils.drawProjectileMotion("predictedShot/withRobotVelocity", initialPosition, estimatedExitVelocity);
    }
    
    
}
