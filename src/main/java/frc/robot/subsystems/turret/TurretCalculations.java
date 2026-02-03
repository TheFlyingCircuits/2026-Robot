package frc.robot.subsystems.turret;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

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

    public static double getAimerTargetDegrees(Translation2d targetFeildRelative, Translation2d turretTranslation) {
        Translation2d targetTranslationTurretRelative = targetFeildRelative.minus(turretTranslation);
        return targetTranslationTurretRelative.getAngle().getDegrees();
    }

    // in the list 0 is output velocity, 1 is launch angle degrees, and 2 is time of impact seconds
    public static double[] getValuesAngleOfAttackOnTheMove(Translation3d originalTarget, ChassisSpeeds robotVelocityFeildRelative, 
    double angleOfAttackDegrees, Translation3d turretPose) {

        Translation3d shootOnTheMoveTarget = targetMovementCompensation(originalTarget, robotVelocityFeildRelative, angleOfAttackDegrees, turretPose);
        return angleOfAttackTrajCalc(shootOnTheMoveTarget, angleOfAttackDegrees, turretPose);
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
        double aimerTargetDegrees = getAimerTargetDegrees(shootOnTheMoveTarget.toTranslation2d(), turretPose.toTranslation2d());



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
    
}
