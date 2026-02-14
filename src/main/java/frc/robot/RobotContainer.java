// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimAndShoot;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.SwerveModuleIOKraken;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretCalculations;
import frc.robot.subsystems.turret.aimer.AimerIOKraken;
import frc.robot.subsystems.turret.aimer.AimerIOSim;
import frc.robot.subsystems.turret.flywheels.FlywheelsIOKraken;
import frc.robot.subsystems.turret.flywheels.FlywheelsIOSim;
import frc.robot.subsystems.turret.hood.HoodIOKraken;
import frc.robot.subsystems.turret.hood.HoodIOSim;


public class RobotContainer {

    public final Drivetrain drivetrain;
    public final Leds leds;
    public final Turret turret;
    public final Indexer indexer;
    public final Intake intake;
    
    protected final HumanDriver duncan = new HumanDriver(0);
    final CommandXboxController duncanController;
    
    private Translation3d aimingTarget;
    private boolean driverReadyToShoot;

    private PathPlannerAuto path;


    public RobotContainer() {
        aimingTarget = TurretCalculations.getHubShootingTargetTranslation();
        /**** INITIALIZE SUBSYSTEMS ****/
        if (RobotBase.isReal()) {
            // NOODLE OFFSETS: FL -0.184814453125, FR 0.044677734375, BL -0.3349609375, BR 0.088134765625 
            drivetrain = new Drivetrain( 
                new GyroIOPigeon(),
                new SwerveModuleIOKraken(0, 1, -0.377686, 0, "FL"), 
                new SwerveModuleIOKraken(2, 3, 0.397705, 1, "FR"),
                new SwerveModuleIOKraken(4, 5, 0.238281, 2, "BL"),
                new SwerveModuleIOKraken(6, 7,  -0.370850, 3, "BR") 
            );
            // drivetrain = new Drivetrain(
            //     new GyroIOSim(){},
            //     new SwerveModuleIOSim(){},
            //     new SwerveModuleIOSim(){},
            //     new SwerveModuleIOSim(){},
            //     new SwerveModuleIOSim(){}
            // );
            leds = new Leds(0,60);
            // drivetrain is only used for getPoseMeters method and does not do anything else
            turret = new Turret(new AimerIOKraken(drivetrain), new FlywheelsIOKraken(), new HoodIOKraken());
            indexer = new Indexer();
            intake = new Intake(new IntakeIOKraken());
        } else {
            drivetrain = new Drivetrain(
                new GyroIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){}
            );
            turret = new Turret(new AimerIOSim(drivetrain), new FlywheelsIOSim(), new HoodIOSim(drivetrain));
            leds = new Leds(0,60);
            indexer = new Indexer();
            intake = new Intake(new IntakeIOSim(drivetrain));
        }
        FlyingCircuitUtils.putNumberOnDashboard("target Turret Deg", 0.0);
        duncanController = duncan.getXboxController();
        new EventTrigger("intake").whileTrue(intake.intakeRunRollersCommand());
        NamedCommands.registerCommand("aim",aimAndShoot( () -> TurretCalculations.possibeTargets.hub, () -> false));
        NamedCommands.registerCommand("aimAndShoot",aimAndShoot( () -> TurretCalculations.possibeTargets.hub, () -> true));
        path = new PathPlannerAuto("LoopAndShootAuto");
        configureBindings();
        setDefaultCommands();
    }

    private void configureBindings() {
        // duncanController.y().onTrue(reSeedRobotPose());
        // duncanController.a().whileTrue(new InstantCommand(() -> VisionIOPhotonLib.acceptAllTags()));
        // duncanController.x().whileTrue(new InstantCommand(() -> VisionIOPhotonLib.acceptNoTags()));
        // duncanController.b().whileTrue(new InstantCommand(() -> VisionIOPhotonLib.onlyAcceptOneTag(10)));
        // duncanController.a().whileTrue(turret.aimAtTargetCommand(() -> FlyingCircuitUtils.getNumberFromDashboard("target Turret Deg", 0.0)));
        // duncanController.b().whileTrue(turret.setAimerVoltsCommand(() -> FlyingCircuitUtils.getNumberFromDashboard("target Turret Deg", 0.0)));
        // duncanController.a().onTrue(turret.aimAtTargetCommand(() -> FlyingCircuitUtils.getNumberFromDashboard("target Turret Deg", 0.0)));
        // duncanController.a().onTrue(turret.aimAtTargetCommand(() -> TurretCalculations.getAimerTargetDegreesRobotToTarget(FieldElement.HUB.getLocation().toTranslation2d(),
        //     drivetrain.getPoseMeters().getTranslation())));
        // duncanController.b().onTrue(Commands.run(() -> TurretCalculations.logShootingFunctions(
        //     FieldElement.HUB.getLocation().plus(new Translation3d(0,0,0.7)), drivetrain.getFieldOrientedVelocity(), -45.0, 
        //         new Translation3d(drivetrain.getPoseMeters().getTranslation().getX(), 
        //             drivetrain.getPoseMeters().getTranslation().getY(), 0.0))));
        // duncanController.b().onTrue(new AimAndShoot(turret, indexer, () -> new Translation3d(drivetrain.getPoseMeters().getTranslation().getX(), 
        //     drivetrain.getPoseMeters().getTranslation().getY(), 0.0), () -> drivetrain.getFieldOrientedVelocity(),
        //     () -> FieldElement.HUB.getLocation().plus(new Translation3d(0,0,0.7)), () -> false, ()-> -45.0));

        // duncanController.x().onTrue(new AimAndShoot(turret, indexer, () -> new Translation3d(drivetrain.getPoseMeters().getTranslation().getX(), 
        //     drivetrain.getPoseMeters().getTranslation().getY(), 0.0), () -> drivetrain.getFieldOrientedVelocity(),
        //     () -> FieldElement.HUB.getLocation().plus(new Translation3d(0,0,0.7)), () -> true, ()-> -45.0));
        duncanController.rightStick().onTrue(aimAndShoot(() -> TurretCalculations.possibeTargets.hub, () -> false));
        duncanController.leftStick().onTrue(aimAndShoot(() -> TurretCalculations.possibeTargets.passing, () -> false));
        duncanController.rightBumper().whileTrue(aimAndShoot(() -> TurretCalculations.currentTarget, () -> true))
        .onFalse(aimAndShoot(() -> TurretCalculations.currentTarget, () -> false));
    }

    public Command getAutonomousCommand() {
        return path;
    }

    public void setDefaultCommands() {
        drivetrain.setDefaultCommand(driverFullyControlDrivetrain().withName("driveDefualtCommand"));
        leds.setDefaultCommand(leds.heartbeatCommand(1.).ignoringDisable(true).withName("ledsDefaultCommand"));
        intake.setDefaultCommand(intake.intakeDefaultCommand());

    }

    private Command aimAndShoot(Supplier<TurretCalculations.possibeTargets> target, Supplier<Boolean> driverReadyToShoot) {
        return new AimAndShoot(turret, indexer, () -> TurretCalculations.getTurretTranslation(drivetrain.getPoseMeters().getTranslation()), 
        () -> drivetrain.getFieldOrientedVelocity(), target, driverReadyToShoot);
    }

    private Command driverFullyControlDrivetrain() { return drivetrain.run(() -> {
        drivetrain.fieldOrientedDrive(duncan.getRequestedFieldOrientedVelocity());
        Logger.recordOutput("drivetrain/runningDefaultCommand", true);
        }).finallyDo(() -> {
            Logger.recordOutput("drivetrain/runningDefaultCommand", false);
        }).withName("driverFullyControlDrivetrain");
    }

    private Command reSeedRobotPose() {return Commands.run(() -> {
        drivetrain.fullyTrustVisionNextPoseUpdate();
        drivetrain.allowTeleportsNextPoseUpdate();
    }).until(drivetrain::seesAcceptableTag).ignoringDisable(true);}

}
