package frc.robot.subsystems.turret.aimer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class AimerIOSim implements AimerIO{
    private Drivetrain drivetrain;
    private double deltaT = 0.02;

    public AimerIOSim(Drivetrain drivetrain) {
        this.drivetrain=drivetrain;
    }

    double simulatedPositionDegrees = 0.0;
    double pivotVelDegPerSec = 0;

    
    @Override
    public void updateInputs(AimerIOInputs inputs) {


        inputs.aimerPositionDegrees += pivotVelDegPerSec * deltaT;
        simulatedPositionDegrees = inputs.aimerPositionDegrees;
        inputs.aimerVelocityDegreesPerSecond = pivotVelDegPerSec;

        double robotYawRad = drivetrain.getPoseMeters().getRotation().getRadians();

        Translation2d aimerTranslation2d = drivetrain.getPoseMeters().getTranslation().plus(new Translation2d(Math.cos(robotYawRad)* 0.4,Math.sin(robotYawRad)*0.4));

        Pose3d aimerPoseOnRobotSIM = new Pose3d(new Translation3d(aimerTranslation2d.getX(),aimerTranslation2d.getY(),0.75), 
            new Rotation3d(0,0,drivetrain.getPoseMeters().getRotation().getRadians()+Units.degreesToRadians(inputs.aimerPositionDegrees)));

        Logger.recordOutput("aimerInputs/aimerPoseOnRobotSIM", aimerPoseOnRobotSIM);
    }

    @Override
    public void setAimerVolts(double volts) {

        pivotVelDegPerSec = volts * 45.0; // volts to deg
    }

    @Override
    public void setTargetAimerPosition(double targetPositionDegrees) {
        targetPositionDegrees = targetPositionDegrees - drivetrain.getPoseMeters().getRotation().getDegrees();
        double simVoltageOutput;
        if(Math.abs(targetPositionDegrees-simulatedPositionDegrees) > 10){
            simVoltageOutput = 0.2 * (targetPositionDegrees-simulatedPositionDegrees);
        } else {
            simVoltageOutput = 0.4 * (targetPositionDegrees-simulatedPositionDegrees);
        }
        setAimerVolts(simVoltageOutput);
    }

}
