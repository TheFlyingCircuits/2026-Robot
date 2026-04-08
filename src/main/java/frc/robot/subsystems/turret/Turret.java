package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
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
        flywheels.updateInputs(flywheelsInputs);
        hood.updateInputs(hoodInputs);
        Logger.processInputs("aimerInputs", aimerInputs);
        Logger.processInputs("flywheelsInputs", flywheelsInputs);
        Logger.processInputs("hoodInputs", hoodInputs);
        Logger.recordOutput("flywheels/avgSurfaceSpeed", this.getAvgFlywheelSurfaceSpeedMetersPerSecond());
    }

    public double getHoodAngleDeg() {
        return hoodInputs.hoodPositionDegrees;
    }

    public double getAimerAngleDeg_robotCoords() {
        return aimerInputs.aimerPositionDegrees;
    }

    public double getAvgFlywheelSurfaceSpeedMetersPerSecond() {
        return (flywheelsInputs.frontWheelVelocityMPS + flywheelsInputs.hoodWheelVelocityMPS) / 2.0;
    }

    public void aimAtTarget(double targetAngleDegreesRobotToTarget) {
        aimer.setTargetAimerPosition(targetAngleDegreesRobotToTarget);
        Logger.recordOutput("aimerInputs/targetAngleDegreesRobotVector", targetAngleDegreesRobotToTarget);
    }

    public void setAimerVolts(double volts) {
        aimer.setAimerVolts(volts);
    }

    public void setAllVolts(double aimerVolts, double hoodVolts, double mainWheelVolts, double hoodWheelVolts) {
        aimer.setAimerVolts(aimerVolts);
        hood.setHoodVolts(hoodVolts);
        flywheels.setFrontWheelVolts(mainWheelVolts);
        flywheels.setHoodWheelVolts(hoodWheelVolts);
    }

    // will return a boolean for each part of turret ready to shoot
    // 0 is aimer ready, 1 is hood ready, 2 is mainFlywheel ready, and 3 is hoodFlywheel ready
    public Supplier<Boolean>[] isReadyToShoot(double aimerToleranceDegrees, double hoodToleranceDegrees, 
    double mainWheelToleranceMPS, double hoodWheelToleranceMPS) {

        Supplier<Boolean> aimerReady = () -> Math.abs(Rotation2d.fromDegrees(aimerInputs.aimerTargetPositionDegrees).minus(Rotation2d.fromDegrees(aimerInputs.aimerPositionDegrees))
        .getDegrees()) <= aimerToleranceDegrees;
        Supplier<Boolean> hoodReady = () ->Math.abs(hoodInputs.targetHoodPositionDegrees-hoodInputs.hoodPositionDegrees) <= hoodToleranceDegrees;
        Supplier<Boolean> mainWheelReady = () ->flywheelsInputs.targetFrontWheelVelocityMPS-flywheelsInputs.frontWheelVelocityMPS <= mainWheelToleranceMPS;
        Supplier<Boolean> hoodWheelReady = () ->flywheelsInputs.targetHoodWheelVelocityMPS-flywheelsInputs.hoodWheelVelocityMPS <= hoodWheelToleranceMPS;
        Supplier<Boolean>[] outputList = new Supplier[4];
        outputList[0] = aimerReady;
        outputList[1] = hoodReady;
        outputList[2] = mainWheelReady;
        outputList[3] = hoodWheelReady;
        return outputList;
    }

    public void setHoodWheelSpeed(double velocityRPS) {
        flywheels.setTargetHoodWheelVelocity(velocityRPS);
    }

    public void aimAtTargetNoShoot(double targetAimerDegrees) {
        aimAtTarget(targetAimerDegrees);
        hood.setTargetHoodPosition(TurretConstants.hoodDefaultAngleDegrees);
        flywheels.setFrontWheelVolts(0.0);
        flywheels.setHoodWheelVolts(0.0);
    }

    public void aimAtTargetAndShoot(double targetAimerDegrees, double targetHoodAngleDegrees, double targetVelocityMetersPerSecondMain, 
        double targetHoodMPS) {
        aimAtTarget(targetAimerDegrees);
        hood.setTargetHoodPosition(targetHoodAngleDegrees);

        // Convert meters per second to RPS
        double frontWheelRPSOutput = targetVelocityMetersPerSecondMain/(Math.PI*TurretConstants.mainFlywheelDiameterMeters);
        flywheels.setTargetFrontWheelVelocity(frontWheelRPSOutput);

        double hoodWheelRPSOutput = (targetHoodMPS)/(Math.PI*TurretConstants.hoodFlywheelDiameterMeters);
        flywheels.setTargetHoodWheelVelocity(hoodWheelRPSOutput);
    }

    public void setWheelsAmps(double mainWheelAmps, double hoodWheelAmps) {
        flywheels.setFrontWheelAmps(mainWheelAmps);
        flywheels.setHoodWheelAmps(hoodWheelAmps);
    }



    public Command aimAtTargetCommand(Supplier<Double> targetAngleDegrees) {
        return this.run(() -> aimAtTarget(targetAngleDegrees.get()));
    }
    
    public Command setAimerVoltsCommand(Supplier<Double> volts) {
        return this.run(() -> setAimerVolts(volts.get()));
    }

    public Command setAimerAmpsCommand(Supplier<Double> amps) {
        return this.run(() -> aimer.setAimerAmps(amps.get()));
    }


    public Command aimAtTargetNoShootCommand(Supplier<Double> targetAimerDegrees) {
        return this.run(() -> aimAtTargetNoShoot(targetAimerDegrees.get()));
    }

    // public Command aimAtTargetAndShootCommand(Supplier<Double> targetAimerDegrees, 
    // Supplier<Double> targetHoodAngleDegrees, Supplier<Double> targetVelocityMetersPerSecond) {
        
    //     return this.run(() -> aimAtTargetAndShoot(targetAimerDegrees.get(),targetHoodAngleDegrees.get(),
    //         targetVelocityMetersPerSecond.get()));
    // }

    public Command turretStopDoingStuffCommand() {
        return this.run(() -> setAllVolts(0.0,0.0,0.0,0.0));
    }

    public Command setAllVoltsCommand(Supplier<Double> aimerVolts, Supplier<Double> hoodVolts, 
        Supplier<Double> mainWheelVolts, Supplier<Double> hoodWheelVolts) {

        return this.run(() -> setAllVolts(aimerVolts.get(), hoodVolts.get(), 
        mainWheelVolts.get(), hoodWheelVolts.get()));
    }

    public Command setWheelsAmpsCommand(Supplier<Double> mainWheelAmps,
     Supplier<Double> hoodWheelAmps) {
        return this.run(() -> setWheelsAmps(mainWheelAmps.get(), hoodWheelAmps.get()));
    }
}
