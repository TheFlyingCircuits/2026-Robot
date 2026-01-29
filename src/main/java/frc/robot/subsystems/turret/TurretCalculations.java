package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class TurretCalculations {
    private static double g = 9.81;

    // This will return two doubles in a list and the first one will be target fuel exit velocity,
    // and second will be target shot angle/hood angle
    // https://www.desmos.com/calculator/uvsbgxh2mb
    public static double[] angleOfAttackTrajCalc(Translation3d target, double angleOfAttackDegrees, Translation3d turretPose) {
        Translation3d targetTranslationTurretRelative = target.minus(turretPose);

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
    double angleOfAttackDegrees, Translation3d turretPose) {
        // see https://www.desmos.com/3d/783cec8449
        Translation3d target = originalTarget;
        for (int approximationCount = 0; approximationCount < 3; approximationCount += 1) {
            // in the list 0 is output velocity, 1 is launch angle degrees, and 2 is time of impact seconds
            double [] trajectoryOuputs = angleOfAttackTrajCalc(target, angleOfAttackDegrees, turretPose);
            double timeOfImpactSeconds = trajectoryOuputs[2];
            Translation3d robotVelocityVector = new Translation3d(robotVelocityFeildRelative.vxMetersPerSecond, robotVelocityFeildRelative.vyMetersPerSecond, 0);
            // robotVelocityVector = robotVelocityVector.times(fudgeFactor);
            target = originalTarget.minus(robotVelocityVector.times(timeOfImpactSeconds));
        }
        return target;
    }

    // in the list 0 is output velocity, 1 is launch angle degrees, and 2 is time of impact seconds
    public static double[] getValuesAngleOfAttackOnTheMove(Translation3d originalTarget, ChassisSpeeds robotVelocityFeildRelative, 
    double angleOfAttackDegrees, Translation3d turretPose) {

        Translation3d shootOnTheMoveTarget = targetMovementCompensation(originalTarget, robotVelocityFeildRelative, angleOfAttackDegrees, turretPose);
        return angleOfAttackTrajCalc(shootOnTheMoveTarget, angleOfAttackDegrees, turretPose);
    }
    
}
