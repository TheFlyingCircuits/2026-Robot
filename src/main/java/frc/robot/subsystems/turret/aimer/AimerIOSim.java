package frc.robot.subsystems.turret.aimer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class AimerIOSim implements AimerIO{
    private Drivetrain drivetrain;
    private double deltaT = 0.02;

    public AimerIOSim(Drivetrain drivetrain) {
        this.drivetrain=drivetrain;
    }

    double simulatedPositionDegrees = 90;
    double pivotVelDegPerSec = 0;

    
    @Override
    public void updateInputs(AimerIOInputs inputs) {


        // inputs.pivotAngleDegrees += pivotVelDegPerSec * deltaT;
        // inputs.pivotAngleRadians += Units.degreesToRadians(pivotVelDegPerSec * deltaT);
        // // System.out.println(inputs.pivotAngleDegrees);
        // inputs.pivotVelocityDegreesPerSecond = pivotVelDegPerSec;

        // double robotYawRad = drivetrain.getPoseMeters().getRotation().getRadians();

        // Translation2d intakeTranslation2d = drivetrain.getPoseMeters().getTranslation().plus(new Translation2d(Math.cos(robotYawRad)* 0.4,Math.sin(robotYawRad)*0.4));

        // Pose3d intakePoseOnRobotSIM = new Pose3d(new Translation3d(intakeTranslation2d.getX(),intakeTranslation2d.getY(),0.3), 
        //     new Rotation3d(0,-inputs.pivotAngleRadians,drivetrain.getPoseMeters().getRotation().getRadians()));

        // Logger.recordOutput("intakeInputs/intakePoseOnRobotSIM", intakePoseOnRobotSIM);
    }

    @Override
    public void setAimerVolts(double volts) {

        pivotVelDegPerSec = volts * 45.0; // volts to deg
    }

    @Override
    public void setTargetAimerPosition(double targetPositionDegrees) {
        double simVoltageOutput = 0.03 * (targetPositionDegrees-simulatedPositionDegrees);
        setAimerVolts(targetPositionDegrees);
    }

}
