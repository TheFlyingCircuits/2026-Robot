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
    public boolean shouldChangeIMUMode = false;
    public int imuMode = 0;

    public Limelights(ArrayList<String> camNames) {
        this.camNames=camNames;
        LimelightHelpers.setPipelineIndex(camNames.get(0), 0);
    }

    public List<SingleTagPoseObservation> getFreshPoseObservations(boolean usingMT2, Drivetrain drivetrain) {
        ArrayList<SingleTagPoseObservation> poseObservations = new ArrayList<SingleTagPoseObservation>();

        for(String camName : camNames) {
            LimelightHelpers.SetRobotOrientation(
                camName, drivetrain.getPoseMeters().getRotation().getDegrees(), 
                Units.radiansToDegrees(drivetrain.getRobotRelativeVelocityMPS().omegaRadiansPerSecond), 0, 0, 0, 0);

            if(usingMT2) {
                LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camName);
        
                if(mt2.tagCount == 1 && mt2.rawFiducials.length == 1) {
                    RawFiducial fiducialUsed = mt2.rawFiducials[0];
                    SingleTagPoseObservation poseObservation = new SingleTagPoseObservation(
                        camName, new Pose3d(mt2.pose), mt2.timestampSeconds, fiducialUsed.id, 
                            fiducialUsed.distToCamera, 0.0, false);

                    poseObservations.add(poseObservation);
                }
            } else {
                LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(camName);
        
                if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                    RawFiducial fiducialUsed = mt1.rawFiducials[0];
                    SingleTagPoseObservation poseObservation = new SingleTagPoseObservation(
                        camName, new Pose3d(mt1.pose), mt1.timestampSeconds, fiducialUsed.id, 
                            fiducialUsed.distToCamera, fiducialUsed.ambiguity, false);

                    poseObservations.add(poseObservation);
                }
            }

        }

        // after we get poses and also set the robot pose for each cam change imu mode if queued
        if(shouldChangeIMUMode) {
            for(String camName : camNames) {
                LimelightHelpers.SetIMUMode(camName, imuMode);
            }
            shouldChangeIMUMode = false;
        }

        return poseObservations;
    }

    public void setIMUModeNextLoop(int mode) {
        imuMode = mode;
        shouldChangeIMUMode = true;
    } 

    public void setIMUModeNow(int mode) {
        for(String camName : camNames) {
            LimelightHelpers.SetIMUMode(camName, imuMode);
        }
    }

}
