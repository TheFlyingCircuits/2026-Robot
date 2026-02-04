package frc.robot.subsystems.turret.hood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.aimer.AimerIO.AimerIOInputs;

public class HoodIOSim implements HoodIO{
    private Drivetrain drivetrain;
    private double deltaT = 0.02;

    public HoodIOSim(Drivetrain drivetrain) {
        this.drivetrain=drivetrain;
    }

    double simulatedPositionDegrees = 0.0;
    double pivotVelDegPerSec = 0;
    double targetHoodDegrees = 0.0;

    
    @Override
    public void updateInputs(HoodIOInputs inputs) {


        inputs.hoodPositionDegrees += pivotVelDegPerSec * deltaT;
        simulatedPositionDegrees = inputs.hoodPositionDegrees;
        inputs.hoodVelocityDegreesPerSecond = pivotVelDegPerSec;

        inputs.targetHoodPositionDegrees = targetHoodDegrees;

        // double robotYawRad = drivetrain.getPoseMeters().getRotation().getRadians();

        // Translation2d hoodTranslation2d = drivetrain.getPoseMeters().getTranslation().plus(new Translation2d(Math.cos(robotYawRad)* 0.4,Math.sin(robotYawRad)*0.4));

        // Pose3d hoodPoseOnRobotSIM = new Pose3d(new Translation3d(hoodTranslation2d.getX(),hoodTranslation2d.getY(),0.75), 
        //     new Rotation3d(0,0,drivetrain.getPoseMeters().getRotation().getRadians()+Units.degreesToRadians(inputs.hoodPositionDegrees)));

        // Logger.recordOutput("hoodInputs/hoodPoseOnRobotSIM", hoodPoseOnRobotSIM);
    }

    @Override
    public void setHoodVolts(double volts) {

        pivotVelDegPerSec = volts * 45.0; // volts to deg
    }

    @Override
    public void setTargetHoodPosition(double targetPositionDegrees) {
        // double targetPositionDegreesTurretRelative = targetPositionDegrees - drivetrain.getPoseMeters().getRotation().getDegrees();

        targetHoodDegrees=targetPositionDegrees;
        // Logger.recordOutput("hoodInputs/targetAngleDegr", targetPositionDegreesTurretRelative);
        double simVoltageOutput;
        if(Math.abs(targetPositionDegrees-simulatedPositionDegrees) > 10){
            simVoltageOutput = 0.4 * (targetPositionDegrees-simulatedPositionDegrees);
        } else {
            simVoltageOutput = 0.8 * (targetPositionDegrees-simulatedPositionDegrees);
        }
        setHoodVolts(simVoltageOutput);
    }
}
