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

        // fake vlaues
        TurretConstants.velocityLookUp.put(0.0,0.0);
        TurretConstants.velocityLookUp.put(1.0,1.2);
        TurretConstants.velocityLookUp.put(2.0,2.3);
        TurretConstants.velocityLookUp.put(3.0,3.4);
        TurretConstants.velocityLookUp.put(4.0,4.5);
        TurretConstants.velocityLookUp.put(5.0,5.7);
        TurretConstants.velocityLookUp.put(5.5,6.3);
        TurretConstants.velocityLookUp.put(6.419948,7.7);

        // realer values 
        TurretConstants.velocityLookUp.put(7.059305,8.7);
        TurretConstants.velocityLookUp.put(7.54,9.7);
        TurretConstants.velocityLookUp.put(7.921,10.8);
        TurretConstants.velocityLookUp.put(8.359459,11.5);
        TurretConstants.velocityLookUp.put(8.974041,12.7);
        TurretConstants.velocityLookUp.put(9.5,13.42154);
        
        TurretConstants.velocityLookUp.put(10.0,14.58644);
        TurretConstants.velocityLookUp.put(10.5,15.7972);
        TurretConstants.velocityLookUp.put(11.0,17.05382);
        TurretConstants.velocityLookUp.put(11.5,18.35631);
        TurretConstants.velocityLookUp.put(12.0,19.70466);
        TurretConstants.velocityLookUp.put(12.5,21.09887);
        TurretConstants.velocityLookUp.put(13.0,22.53894);
        TurretConstants.velocityLookUp.put(13.5, 24.02487);
        // TurretConstants.velocityLookUp.put(7.059305,8.559305);

        // TurretConstants.velocityLookUp.put(7.059305,8.559305);


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
        Logger.recordOutput("Aimer error deg", aimerInputs.aimerTargetPositionDegrees-aimerInputs.aimerPositionDegrees);
    }

    public double getHoodAngleDeg() {
        return hoodInputs.hoodPositionDegrees;
    }

    public double getAimerAngleDeg_robotCoords() {
        return aimerInputs.aimerPositionDegrees;
    }

    public double getAvgFlywheelSurfaceSpeedMetersPerSecond() {
        return (flywheelsInputs.frontWheelVelocityMPS);
    }

    public void aimAtTarget(double targetAngleDegreesRobotToTarget) {
        aimer.setTargetAimerPosition(targetAngleDegreesRobotToTarget);
        Logger.recordOutput("aimerInputs/targetAngleDegreesRobotVector", targetAngleDegreesRobotToTarget);
    }

    public void setAimerVolts(double volts) {
        aimer.setAimerVolts(volts);
    }

    public void setAllVolts(double aimerVolts, double hoodVolts, double mainWheelVolts) {
        aimer.setAimerVolts(aimerVolts);
        hood.setHoodVolts(hoodVolts);
        flywheels.setFrontWheelVolts(mainWheelVolts);
        // flywheels.setHoodWheelVolts(hoodWheelVolts);
    }

    // will return a boolean for each part of turret ready to shoot
    // 0 is aimer ready, 1 is hood ready, 2 is mainFlywheel ready, and 3 is hoodFlywheel ready
    public Supplier<Boolean>[] isReadyToShoot(double aimerToleranceDegrees, double hoodToleranceDegrees, 
    double mainWheelToleranceMPS) {

        Supplier<Boolean> aimerReady = () -> Math.abs(Rotation2d.fromDegrees(aimerInputs.aimerTargetPositionDegrees).minus(Rotation2d.fromDegrees(aimerInputs.aimerPositionDegrees))
        .getDegrees()) <= aimerToleranceDegrees;
        Supplier<Boolean> hoodReady = () ->Math.abs(hoodInputs.targetHoodPositionDegrees-hoodInputs.hoodPositionDegrees) <= hoodToleranceDegrees;
        Supplier<Boolean> mainWheelReady = () ->flywheelsInputs.targetFrontWheelVelocityMPS-flywheelsInputs.frontWheelVelocityMPS <= mainWheelToleranceMPS;
        // Supplier<Boolean> hoodWheelReady = () ->flywheelsInputs.targetHoodWheelVelocityMPS-flywheelsInputs.hoodWheelVelocityMPS <= hoodWheelToleranceMPS;
        Supplier<Boolean>[] outputList = new Supplier[3];
        outputList[0] = aimerReady;
        outputList[1] = hoodReady;
        outputList[2] = mainWheelReady;
        // outputList[3] = hoodWheelReady;
        return outputList;
    }

    // public void setHoodWheelSpeed(double velocityRPS) {
    //     flywheels.setTargetHoodWheelVelocity(velocityRPS);
    // }

    public void aimAtTargetNoShoot(double targetAimerDegrees) {
        aimAtTarget(targetAimerDegrees);
        hood.setTargetHoodPosition(TurretConstants.hoodDefaultAngleDegrees);
        flywheels.setFrontWheelVolts(0.0);
        // flywheels.setHoodWheelVolts(0.0);
    }

    public void aimAtTargetAndShoot(double targetAimerDegrees, double targetHoodAngleDegrees, double targetVelocityMetersPerSecondMain) {
        aimAtTarget(targetAimerDegrees);
        hood.setTargetHoodPosition(targetHoodAngleDegrees);

        // Convert meters per second to RPS
        double frontWheelRPSOutput = targetVelocityMetersPerSecondMain/(Math.PI*TurretConstants.mainFlywheelDiameterMeters);
        flywheels.setTargetFrontWheelVelocity(frontWheelRPSOutput);

        // double hoodWheelRPSOutput = (targetHoodMPS)/(Math.PI*TurretConstants.hoodFlywheelDiameterMeters);
        // flywheels.setTargetHoodWheelVelocity(hoodWheelRPSOutput);
    }

    public void setWheelsAmps(double mainWheelAmps) {
        flywheels.setFrontWheelAmps(mainWheelAmps);
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
        return this.run(() -> setAllVolts(0.0,0.0,0.0));
    }

    public Command setAllVoltsCommand(Supplier<Double> aimerVolts, Supplier<Double> hoodVolts, 
        Supplier<Double> mainWheelVolts) {

        return this.run(() -> setAllVolts(aimerVolts.get(), hoodVolts.get(), 
        mainWheelVolts.get()));
    }

    public Command setWheelsAmpsCommand(Supplier<Double> mainWheelAmps) {
        return this.run(() -> setWheelsAmps(mainWheelAmps.get()));
    }
}
