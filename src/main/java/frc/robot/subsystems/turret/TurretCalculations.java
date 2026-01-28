package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class TurretCalculations {
    private static double g = 9.81;

    public static Pose3d getMovementCompensatedTarget() {
        return new Pose3d();
    }

    // This will return two doubles in a list and the first one will be target fuel exit velocity,
    // and second will be target shot angle/hood angle
    // https://www.desmos.com/calculator/uvsbgxh2mb
    public static Translation3d angleOfAttackTrajCalc(Translation3d target, double angleOfAttackDegrees, Translation3d turretPose) {
        Translation3d targetTranslationTurretRelative = target.minus(turretPose);

        double ballDisplacementXYMeters = Math.sqrt(Math.pow(targetTranslationTurretRelative.getX(), 2)
            + Math.pow(targetTranslationTurretRelative.getY(),2));

        // ballDisplacementXYMeters = targetTranslationTurretRelative.toTranslation2d().getNorm(); is easy way

        double ballDisplacementZMeters = targetTranslationTurretRelative.getZ();

        double timeOfImpact = Math.sqrt((2.0/g)*(ballDisplacementZMeters
            -ballDisplacementXYMeters*Math.tan(Units.degreesToRadians(angleOfAttackDegrees))));

        double initialVelocityXY = ballDisplacementXYMeters/timeOfImpact;
        double initialVelocityZ = (ballDisplacementZMeters+(0.5*g)*Math.pow(timeOfImpact,2))/timeOfImpact;
        // Translation3d ballVelocityVectorTurretRelative = new Translation3d(ballDisplacementXYMeters,,ballDisplacementZMeters);

        return new Translation3d();
    }
}
