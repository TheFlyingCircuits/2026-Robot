package frc.robot.subsystems.vision;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Limelights {
    public ArrayList<String> camNames = new ArrayList<String>();
    public boolean shouldChangeIMUMode = false;
    public int imuMode = 0;
    double targetBoxID = 12; 

    public Limelights(ArrayList<String> camNames) {
        this.camNames=camNames;
        for(String camName : camNames) {
            LimelightHelpers.setPipelineIndex(camName, 0);
            LimelightHelpers.SetIMUAssistAlpha(camName, 0.05);
        }
    }

    public Optional<Pose3d> getBoxTagPose(Pose2d robotPose, Drivetrain drivetrain) {

        for(String camName : camNames) {

            double robotYawRateRad = drivetrain.getRobotRelativeVelocityMPS().omegaRadiansPerSecond;
            LimelightHelpers.SetRobotOrientation(
            camName, drivetrain.getPoseMeters().getRotation().getDegrees(), 
            Units.radiansToDegrees(robotYawRateRad), 0, 0, 0, 0);

            RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(camName);

            for (RawFiducial fiducial : fiducials) {
                    
                // 1. Distance relative to robot in meters
                double distanceM = fiducial.distToRobot; 
                
                // 2. Yaw and pitch relative to camera deg
                double yawToTagDeg = fiducial.txnc;
                double pitchToTagDeg = fiducial.tync;

                                
                // double camYaw = yawToTagDeg;
                // double yawReal = yawToTagDeg;
                // double pitchReal = pitchToTagDeg;

                
                double camYaw = camName.equals("limelight-left") ? 90 : -90;
                double yawReal = camYaw + -yawToTagDeg;
                double pitchReal = pitchToTagDeg+25;

                // System.out.println(yawReal);

                return Optional.ofNullable(new Pose3d(robotPose).plus(new Transform3d(
                    new Translation3d(distanceM * Math.cos(Units.degreesToRadians(yawReal)),
                    distanceM * Math.sin(Units.degreesToRadians(yawReal)),
                    distanceM * Math.sin(Units.degreesToRadians(pitchReal))),
                    // new Rotation3d()

                    new Rotation3d(0,0, Units.degreesToRadians(yawReal+180))
                )));
            }
        }

        return Optional.empty();
    }

    public List<SingleTagPoseObservation> getFreshPoseObservations(boolean usingMT2, Drivetrain drivetrain) {
        ArrayList<SingleTagPoseObservation> poseObservations = new ArrayList<SingleTagPoseObservation>();
        return poseObservations;

        // for(String camName : camNames) {
        //     double robotYawRateRad = drivetrain.getRobotRelativeVelocityMPS().omegaRadiansPerSecond;
        //     LimelightHelpers.SetRobotOrientation(
        //         camName, drivetrain.getPoseMeters().getRotation().getDegrees(), 
        //         Units.radiansToDegrees(robotYawRateRad), 0, 0, 0, 0);

        //     if(usingMT2) {
        //         LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(camName);
        
        //         if(LimelightHelpers.validPoseEstimate(mt2)) {
        //             RawFiducial[] fiducialsUsed = mt2.rawFiducials;
        //             boolean seesMultibleTags = fiducialsUsed.length > 1;

        //             RawFiducial closestTag = fiducialsUsed[0];
        //             for(RawFiducial tag : fiducialsUsed) {
        //                 if(tag.distToCamera < closestTag.distToCamera) closestTag = tag;
        //             }

        //             SingleTagPoseObservation poseObservation = new SingleTagPoseObservation(
        //                 camName, new Pose3d(mt2.pose), mt2.timestampSeconds, fiducialsUsed, 
        //                     closestTag.distToCamera, 0.0, seesMultibleTags, closestTag.id, robotYawRateRad);

        //             poseObservations.add(poseObservation);
        //         }
        //     } else {
        //         LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(camName);
        
        //         if(LimelightHelpers.validPoseEstimate(mt1)) {
        //             RawFiducial[] fiducialsUsed = mt1.rawFiducials;
        //             boolean seesMultibleTags = fiducialsUsed.length > 1;

        //             RawFiducial closestTag = fiducialsUsed[0];
        //             for(RawFiducial tag : fiducialsUsed) {
        //                 if(tag.distToCamera < closestTag.distToCamera) closestTag = tag;
        //             }

        //             SingleTagPoseObservation poseObservation = new SingleTagPoseObservation(
        //                 camName, new Pose3d(mt1.pose), mt1.timestampSeconds, fiducialsUsed, 
        //                     closestTag.distToCamera, closestTag.ambiguity, seesMultibleTags, closestTag.id, robotYawRateRad);

        //             poseObservations.add(poseObservation);
        //         }
        //     }

        // }

        // // after we get poses and also set the robot pose for each cam change imu mode if queued
        // if(shouldChangeIMUMode) {
        //     for(String camName : camNames) {
        //         LimelightHelpers.SetIMUMode(camName, imuMode);
        //     }
        //     shouldChangeIMUMode = false;
        // }

        // return poseObservations;
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
