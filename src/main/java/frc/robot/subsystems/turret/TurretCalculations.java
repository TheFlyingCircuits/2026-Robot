package frc.robot.subsystems.turret;

import java.util.ArrayList;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.FlyingCircuitUtils;
import frc.robot.PlayingField.FieldElement;

public class TurretCalculations {
    private static double g = 9.81;

    // This will return two doubles in a list and the first one will be target fuel exit velocity,
    // and second will be target shot angle/hood angle
    // https://www.desmos.com/calculator/uvsbgxh2mb
    public static double[] angleOfAttackTrajCalc(Translation3d targetTranslation, double angleOfAttackDegrees, Translation3d turretTranslation) {
        Translation3d targetTranslationTurretRelative = targetTranslation.minus(turretTranslation);

        double ballDisplacementXYMeters = Math.sqrt(Math.pow(targetTranslationTurretRelative.getX(), 2)
            + Math.pow(targetTranslationTurretRelative.getY(),2)); // same as math.hypot but I wanted to write it out one time

        // ballDisplacementXYMeters = targetTranslationTurretRelative.toTranslation2d().getNorm(); is easy way

        double ballDisplacementZMeters = targetTranslationTurretRelative.getZ();

        double timeOfImpactSeconds = Math.sqrt((2.0/g)*(ballDisplacementZMeters
            -ballDisplacementXYMeters*Math.tan(Units.degreesToRadians(angleOfAttackDegrees))));

        double initialVelocityXYMetersPerSecond = ballDisplacementXYMeters/timeOfImpactSeconds;
        double initialVelocityZMetersPerSecond = (ballDisplacementZMeters+(0.5*g)*Math.pow(timeOfImpactSeconds,2))/timeOfImpactSeconds;
        double exitVelocityMetersPerSecond = Math.hypot(initialVelocityXYMetersPerSecond, initialVelocityZMetersPerSecond);

        double launchAngleRadians = Math.atan2(initialVelocityZMetersPerSecond, initialVelocityXYMetersPerSecond);
        double launchAngleDegrees = Units.radiansToDegrees(launchAngleRadians);

        return new double[]{exitVelocityMetersPerSecond, launchAngleDegrees, timeOfImpactSeconds};
    }



    public static Translation3d targetMovementCompensation(Translation3d originalTarget, ChassisSpeeds robotVelocityFeildRelative, 
    double angleOfAttackDegrees, Translation3d turretTranslation) {
        // see https://www.desmos.com/3d/783cec8449
        Translation3d target = originalTarget;
        for (int approximationCount = 0; approximationCount < 6; approximationCount += 1) {
            // in the list 0 is output velocity, 1 is launch angle degrees, and 2 is time of impact seconds
            double [] trajectoryOuputs = angleOfAttackTrajCalc(target, angleOfAttackDegrees, turretTranslation);
            double timeOfImpactSeconds = trajectoryOuputs[2];
            Translation3d robotVelocityVector = new Translation3d(robotVelocityFeildRelative.vxMetersPerSecond, robotVelocityFeildRelative.vyMetersPerSecond, 0);
            // robotVelocityVector = robotVelocityVector.times(fudgeFactor);
            target = originalTarget.minus(robotVelocityVector.times(timeOfImpactSeconds));
        }
        return target;
    }


    // this returns the angle of turret pose to target but it disregards robot orientation so its essentially robot pose
    // to target and the actual aiming target is converted in the AimerIOKraken file in the setTargetPosition method
    public static double getAimerTargetDegreesRobotToTarget(Translation2d targetFeildRelative, Translation2d turretTranslation) {
        Translation2d targetTranslationTurretRelative = targetFeildRelative.minus(turretTranslation);
        return targetTranslationTurretRelative.getAngle().getDegrees();
    }

    public static void logShootingFunctions(Translation3d originalTarget, ChassisSpeeds robotVelocityFeildRelative, 
    double angleOfAttackDegrees, Translation3d turretPose) {


        ArrayList<Translation3d> fuelTrajPoints = new ArrayList<>();
        ArrayList<Translation3d> fuelTrajPointsWithRobotVelocity = new ArrayList<>();

        Translation3d shootOnTheMoveTarget = targetMovementCompensation(originalTarget, robotVelocityFeildRelative, angleOfAttackDegrees, turretPose);
        double[] shooterValues = angleOfAttackTrajCalc(shootOnTheMoveTarget, angleOfAttackDegrees, turretPose);
        
        double tImpact = shooterValues[2];
        double xyVelocityComponent = shooterValues[0] * Math.cos(Units.degreesToRadians(shooterValues[1]));
        double zVelocityComponent = shooterValues[0] * Math.sin(Units.degreesToRadians(shooterValues[1]));
        double aimerTargetDegrees = getAimerTargetDegreesRobotToTarget(shootOnTheMoveTarget.toTranslation2d(), turretPose.toTranslation2d());



        // get 6 points from traj withoutRobotVelocity
        for(double t = 0.0; t<=tImpact; t= t+(tImpact/6.0)){
            double xyPositionMeters = xyVelocityComponent * t;
            double zPosisionMeters = (zVelocityComponent*t) - (0.5*9.81*Math.pow(t,2));

            Translation3d fuelPositionFeildRelative = new Translation3d((xyPositionMeters*Math.cos(Units.degreesToRadians(aimerTargetDegrees))), 
                (xyPositionMeters*Math.sin(Units.degreesToRadians(aimerTargetDegrees))), zPosisionMeters);
            Translation3d fuelPositionTurretRelative = turretPose.plus(fuelPositionFeildRelative);
            fuelTrajPoints.add(fuelPositionTurretRelative);
        }

        // get 6 points from traj withoutRobotVelocity
        for(double t = 0.0; t<=tImpact; t= t+(tImpact/6.0)){
            double xyPositionMetersNoRobotVelocity = xyVelocityComponent * t;
            double zPosisionMeters = (zVelocityComponent*t) - (0.5*9.81*Math.pow(t,2));

            double xPositionWithRobotVelocity = xyPositionMetersNoRobotVelocity*Math.cos(Units.degreesToRadians(aimerTargetDegrees))
                + robotVelocityFeildRelative.vxMetersPerSecond * t;

            double yPositionWithRobotVelocity = xyPositionMetersNoRobotVelocity*Math.sin(Units.degreesToRadians(aimerTargetDegrees))
                + robotVelocityFeildRelative.vyMetersPerSecond * t;

            Translation3d fuelPositionFeildRelative = new Translation3d(xPositionWithRobotVelocity, yPositionWithRobotVelocity, zPosisionMeters);
            Translation3d fuelPositionTurretRelative = turretPose.plus(fuelPositionFeildRelative);
            fuelTrajPointsWithRobotVelocity.add(fuelPositionTurretRelative);
        }

        Logger.recordOutput("TrajOuputs/fuelTrajPoints", fuelTrajPoints.toArray(new Translation3d[fuelTrajPoints.size()]));
        Logger.recordOutput("TrajOuputs/fuelTrajPointsWithRobotVelocity", fuelTrajPointsWithRobotVelocity.toArray(new Translation3d[fuelTrajPointsWithRobotVelocity.size()]));
        Logger.recordOutput("TrajOuputs/shootOnTheMoveTarget", shootOnTheMoveTarget);
        Logger.recordOutput("TrajOuputs/originalTarget", originalTarget);
    }

    public static Translation3d getHubShootingTargetTranslation() {
        Translation3d shootingPose = FieldElement.HUB.getLocation().plus(new Translation3d(0.0,0.0,0.5));
        return shootingPose;
    }

    public static Translation3d getPassingTargetTranslation(Supplier<Translation2d> robotTranslation) {

        double distanceToLeftTrench = Math.abs(FieldElement.TRENCH_LEFT.getLocation2d().getDistance(robotTranslation.get()));
        double distanceToRightTrench = Math.abs(FieldElement.TRENCH_RIGHT.getLocation2d().getDistance(robotTranslation.get()));
        
        Translation3d target;

        double inverIfRed = FlyingCircuitUtils.getAllianceDependentValue(-1.0, 1.0, 1.0);

        if(distanceToLeftTrench<distanceToRightTrench) {
            target=FieldElement.TRENCH_LEFT.getLocation().plus(new Translation3d(-3.0*inverIfRed,-2.0*inverIfRed,0.5));
        } else {
            target=FieldElement.TRENCH_RIGHT.getLocation().plus(new Translation3d(-3.0*inverIfRed,2.0*inverIfRed,0.5));
        }

        return target;
    }

    // change z based off cad model once done
    public static Translation3d getTurretTranslation(Translation2d robotTranslation) {
        return new Translation3d(robotTranslation.getX(), robotTranslation.getY(), 0.6);
    }
    
}
