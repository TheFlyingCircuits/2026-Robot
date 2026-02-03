package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.turret.aimer.AimerIO;
import frc.robot.subsystems.turret.aimer.AimerIOInputsAutoLogged;
import frc.robot.subsystems.turret.flywheels.FlywheelsIO;
import frc.robot.subsystems.turret.flywheels.FlywheelsIOInputsAutoLogged;
import frc.robot.subsystems.turret.hood.HoodIO;
import frc.robot.subsystems.turret.hood.HoodIOInputsAutoLogged;

public class Turret extends SubsystemBase{

    AimerIOInputsAutoLogged aimerInputs;
    FlywheelsIOInputsAutoLogged flywheelsInputs;
    HoodIOInputsAutoLogged hoodInputs;

    AimerIO aimer;
    FlywheelsIO flywheels;
    HoodIO hood;

    public Turret(AimerIO aimer, FlywheelsIO flywheels, HoodIO hood) {
        this.aimer = aimer;
        this.flywheels = flywheels;
        this.hood = hood;

        aimerInputs = new AimerIOInputsAutoLogged();
        flywheelsInputs = new FlywheelsIOInputsAutoLogged();
        hoodInputs = new HoodIOInputsAutoLogged();
    }

    @Override 
    public void periodic() {
        aimer.updateInputs(aimerInputs);
        Logger.processInputs("aimerInputs", aimerInputs);
    }

    public void aimAtTarget(double targetAngleDegrees) {
        Logger.recordOutput("aimerInputs/targetAngleDegreesRobotVector", targetAngleDegrees);
        aimer.setTargetAimerPosition(targetAngleDegrees);
    }

    public void setAimerVolts(double volts) {
        aimer.setAimerVolts(volts);
    }

    public void aimAtTargetNoShoot(double targetAimerDegrees) {
        aimAtTarget(targetAimerDegrees);
        hood.setTargetHoodPosition(TurretConstants.hoodDefaultAngleDegrees);
        flywheels.setTargetFrontWheelVelocity(0.0);
        flywheels.setTargetHoodWheelVelocity(0.0);
    }

    public void aimAtTargetAndShoot(double targetAimerDegrees, double targetHoodAngleDegrees, double targetVelocityMetersPerSecond) {
        aimAtTarget(targetAimerDegrees);
        hood.setTargetHoodPosition(targetHoodAngleDegrees);

        // Convert meters per second to RPS
        double frontWheelRPSOutput = targetVelocityMetersPerSecond/(Math.PI*TurretConstants.mainFlywheelDiameterMeters);
        flywheels.setTargetFrontWheelVelocity(frontWheelRPSOutput);

        double hoodWheelRPSOutput = targetVelocityMetersPerSecond/(Math.PI*TurretConstants.hoodFlywheelDiameterMeters);
        flywheels.setTargetHoodWheelVelocity(hoodWheelRPSOutput);
    }

    public Command aimAtTargetCommand(DoubleSupplier targetAngleDegrees) {
        return this.run(() -> aimAtTarget(targetAngleDegrees.getAsDouble()));
    }
    
    public Command setAimerVoltsCommand(DoubleSupplier volts) {
        return this.run(() -> setAimerVolts(volts.getAsDouble()));
    }

    public Command aimAtTargetNoShootCommand(DoubleSupplier targetAimerDegrees) {
        return this.run(() -> aimAtTargetNoShoot(targetAimerDegrees.getAsDouble()));
    }

    public Command aimAtTargetAndShootCommand(DoubleSupplier targetAimerDegrees, 
    DoubleSupplier targetHoodAngleDegrees, DoubleSupplier targetVelocityMetersPerSecond) {
        
        return this.run(() -> aimAtTargetAndShoot(targetAimerDegrees.getAsDouble(),targetHoodAngleDegrees.getAsDouble(),
            targetVelocityMetersPerSecond.getAsDouble()));
    }
}
