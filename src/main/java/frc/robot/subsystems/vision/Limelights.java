package frc.robot.subsystems.vision;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Limelights {
    public ArrayList<String> camNames = new ArrayList<String>();

    public Limelights(ArrayList<String> camNames) {
        this.camNames=camNames;
    }

    public List<SingleTagPoseObservation> getFreshPoseObservations(boolean usingTrig, Drivetrain drivetrain) {
        ArrayList<SingleTagPoseObservation> poseObservations = new ArrayList<SingleTagPoseObservation>();

        for(String camName : camNames) {
            LimelightHelpers.SetRobotOrientation(
                camName, drivetrain.getPoseMeters().getRotation().getDegrees(), 
                Units.radiansToDegrees(drivetrain.getRobotRelativeVelocityMPS().omegaRadiansPerSecond), 0, 0, 0, 0);

            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(camName);
      
            if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                RawFiducial fiducialUsed = mt1.rawFiducials[0];
                SingleTagPoseObservation poseObservation = new SingleTagPoseObservation(
                    camName, new Pose3d(mt1.pose), mt1.timestampSeconds, fiducialUsed.id, 
                        fiducialUsed.distToCamera, fiducialUsed.ambiguity, false);

                poseObservations.add(poseObservation);
            }

        }

        return poseObservations;
    }

}
