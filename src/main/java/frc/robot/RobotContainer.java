// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.PlayingField.FieldElement;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.ShootWithParams;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.SwerveModuleIOKraken;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIOKraken;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOMotors;
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
    // public final Leds leds;
    // public final LedsCANdle canLeds;
    // public final LedsCANdle canLedsCounter;
    public final Turret turret;
    public final Indexer indexer;
    public final Intake intake;

    private final SendableChooser<String> autoChooser;

    
    protected final HumanDriver duncan = new HumanDriver(0);
    final CommandXboxController duncanController;

    public RobotContainer() {
        /**** INITIALIZE SUBSYSTEMS ****/
        if (RobotBase.isReal()) {
            // NOODLE OFFSETS: FL -0.184814453125, FR 0.044677734375, BL -0.3349609375, BR 0.088134765625 
            drivetrain = new Drivetrain( 
                new GyroIOPigeon(),
                new SwerveModuleIOKraken(1, 2, -0.377686, 1, "FL"), 
                new SwerveModuleIOKraken(3, 4, 0.397705, 2, "FR"),
                new SwerveModuleIOKraken(5, 6, 0.238281, 3, "BL"),
                new SwerveModuleIOKraken(7, 8,  -0.370850, 4, "BR") 
            );
            // drivetrain = new Drivetrain(
            //     new GyroIOSim(){},
            //     new SwerveModuleIOSim(){},
            //     new SwerveModuleIOSim(){},
            //     new SwerveModuleIOSim(){},
            //     new SwerveModuleIOSim(){}
            // );
            // leds = new Leds(0,60);
            // drivetrain is only used for getPoseMeters method and does not do anything else
            turret = new Turret(new AimerIOKraken(drivetrain), new FlywheelsIOKraken(), new HoodIOKraken());
            indexer = new Indexer(new IndexerIOKraken());
            intake = new Intake(new IntakeIOMotors());
        } else {
            drivetrain = new Drivetrain(
                new GyroIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){}
            );
            turret = new Turret(new AimerIOSim(drivetrain), new FlywheelsIOSim(), new HoodIOSim(drivetrain));
            // leds = new Leds(0,60);
            indexer = new Indexer(new IndexerIOSim());
            intake = new Intake(new IntakeIOSim(drivetrain));
        }

        // canLeds = new LedsCANdle(0, 60);
        // canLedsCounter = new LedsCANdle(1, 60);

        FlyingCircuitUtils.putNumberOnDashboard("targetAimerDeg", 0.0);
        FlyingCircuitUtils.putNumberOnDashboard("targetHoodDeg", 0.0);
        FlyingCircuitUtils.putNumberOnDashboard("targetMainWheelMPS", 0.0);
        FlyingCircuitUtils.putNumberOnDashboard("targetHoodWheelMPS", 0.0);
        FlyingCircuitUtils.putNumberOnDashboard("kicker", 0.0);
        FlyingCircuitUtils.putNumberOnDashboard("sideKicker", 0.0);
        FlyingCircuitUtils.putNumberOnDashboard("bigSpinner", 0.0);


        duncanController = duncan.getXboxController();

        // AUTO ---------------

        autoChooser = new SendableChooser<>();
        autoChooser.addOption("Deep", "Deep");
        autoChooser.addOption("Shallow", "Shallow");
        autoChooser.addOption("DeepShallow", "DeepShallow");
        autoChooser.addOption("ShallowDeep", "ShallowDeep");

        autoChooser.setDefaultOption("Deep", "Deep");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        new EventTrigger("intakeDown").onTrue(new ProxyCommand(intake.intakeDownCommand().until(() -> intake.isIntakeDown()).andThen(intake.intakeDefualtAndIntakeCommand())));
        new EventTrigger("intake").onTrue(new ProxyCommand(intake.intakeDefualtAndIntakeCommand()));
        // new EventTrigger("aim").whileTrue(aimAndShoot(() -> TurretCalculations.possibeTargets.hub, () -> false, () -> true));
        new EventTrigger("aim").whileTrue(new ProxyCommand(new AimAndShoot(turret, indexer, () -> TurretCalculations.getTurretTranslation(drivetrain.getPoseMeters().getTranslation()), 
        () -> drivetrain.getFieldOrientedVelocity(), () -> TurretCalculations.possibeTargets.hub, () -> true, true)));

        new EventTrigger("shoot").onTrue(new ProxyCommand(aimAndShoot(() -> TurretCalculations.possibeTargets.hub, () -> true, () -> true)));

        Commands.runOnce(drivetrain::setRobotFacingForward);
        configureBindings();
        setDefaultCommands();
    }

    private void configureBindings() {
        // duncanController.rightStick().onTrue(aimAndShoot(() -> TurretCalculations.possibeTargets.hub, () -> false, () -> true));
        // duncanController.leftStick().onTrue(aimAndShoot(() -> TurretCalculations.possibeTargets.passing, () -> false, () -> true));
        // duncanController.rightBumper().whileTrue(aimAndShoot(() -> TurretCalculations.currentTarget, () -> true, () -> true))
        // .onFalse(aimAndShoot(() -> TurretCalculations.currentTarget, () -> false, () -> true));

        // duncanController.leftBumper().whileTrue(intake.intakeDefualtAndIntakeCommand());
        // duncanController.leftTrigger().whileTrue(intake.intakeDefualtAndIntakeCommand().alongWith(driveTowardsFuelTeleop()));

        // duncanController.y().onTrue(reSeedRobotPose());
        // duncanController.start().onTrue(Commands.runOnce(drivetrain::setRobotFacingForward));

        // // reset everything
        duncanController.x().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
        }));

        duncanController.rightBumper().whileTrue(new ShootWithParams(turret, indexer, 
        ()-> FlyingCircuitUtils.getNumberFromDashboard("targetAimerDeg", 0.0),
        ()->FlyingCircuitUtils.getNumberFromDashboard("targetHoodDeg", 0.0),
        ()->FlyingCircuitUtils.getNumberFromDashboard("targetMainWheelMPS", 0.0)));

        // duncanController.rightTrigger().whileTrue(intake.reverseIntakeCommand().alongWith(indexer.reverseIndexerCommand()));

        // duncanController.rightBumper().whileTrue(turret.setAllVoltsCommand(
        // ()-> FlyingCircuitUtils.getNumberFromDashboard("aimerVolts", 0.0),
        // ()->FlyingCircuitUtils.getNumberFromDashboard("hoodVolts", 0.0),
        // ()->FlyingCircuitUtils.getNumberFromDashboard("mainWheelVolts", 0.0),
        // ()->FlyingCircuitUtils.getNumberFromDashboard("hoodWheelVolts", 0.0)))
        // .whileFalse(
        // turret.setAllVoltsCommand(()-> 0.0, ()-> 0.0, ()-> 0.0, ()-> 0.0)
        // );  

        // duncanController.rightBumper().whileTrue(turret.setWheelsAmpsCommand(
        // ()->FlyingCircuitUtils.getNumberFromDashboard("mainWheelAmps", 0.0),
        // ()->FlyingCircuitUtils.getNumberFromDashboard("hoodWheelAmps", 0.0)))
        // .whileFalse(
        // turret.setWheelsAmpsCommand(()-> 0.0, ()-> 0.0)
        // );  


        // duncanController.rightBumper().whileTrue(turret.aimAtTargetAndShootCommand(
        // ()-> FlyingCircuitUtils.getNumberFromDashboard("targetAimerDeg", 0.0),
        // ()->FlyingCircuitUtils.getNumberFromDashboard("targetHoodDeg", 0.0),
        // ()->FlyingCircuitUtils.getNumberFromDashboard("targetMainWheelMPS", 0.0)))
        // .whileFalse(
        // turret.setAllVoltsCommand(()-> 0.0, ()-> 0.0, ()-> 0.0, ()-> 0.0)
        // );  

        

    }

    // public Command normalAuto() {
    //     return new SequentialCommandGroup(AutoBuilder.followPath(PathPlannerPath.fromPathFile("a")).alongWith(AimAndShoot).until());
    // }

    public void setDefaultCommands() {
        // drivetrain.setDefaultCommand(driverFullyControlDrivetrain().withName("driveDefualtCommand"));
        // // leds.setDefaultCommand(leds.heartbeatCommand(1.).ignoringDisable(true).withName("ledsDefaultCommand"));
        turret.setDefaultCommand(turret.turretStopDoingStuffCommand());
        indexer.setDefaultCommand(indexer.stopIndexingCommand());
        // canLedsCounter.setDefaultCommand(canLedsCounter.heartbeatCommand().ignoringDisable(true));
        // intake.setDefaultCommand(intake.noVoltageCommand());

    }

    public Supplier<Boolean> isInAuto() {
        return () -> DriverStation.isAutonomous();
    }

    private Command aimAndShoot(Supplier<TurretCalculations.possibeTargets> target, Supplier<Boolean> driverReadyToShoot, Supplier<Boolean> needsReqs) {
        return new AimAndShoot(turret, indexer, () -> TurretCalculations.getTurretTranslation(drivetrain.getPoseMeters().getTranslation()), 
        () -> drivetrain.getFieldOrientedVelocity(), target, driverReadyToShoot, needsReqs.get());
    }

    // private Command aimAndShoot(Supplier<TurretCalculations.possibeTargets> target, Supplier<Boolean> driverReadyToShoot) {
    //     return new AimAndShoot(turret, indexer, () -> new Translation3d(0,0,0), 
    //     () -> new ChassisSpeeds(0.0,0.0,0.0), target, driverReadyToShoot);
    // }

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


    private Command driveTowardsFuelTeleop() { return drivetrain.run(() -> {
        Optional<Pose3d> fuel = drivetrain.getClosestCluster();
        ChassisSpeeds driverRequest = duncan.getRequestedFieldOrientedVelocity();
        // TODO: tune override ratio
        boolean significantRotationRequested = Math.abs(driverRequest.omegaRadiansPerSecond) > 0.5;
        boolean driverOverridingSelectedFuel = fuel.isPresent() && significantRotationRequested;
        Logger.recordOutput("fuelTracking/driverOverridingSelectedTarget", driverOverridingSelectedFuel);
        if (fuel.isEmpty() || driverOverridingSelectedFuel) {
            drivetrain.fieldOrientedDrive(duncan.getRequestedFieldOrientedVelocity());
            return;
        }


        // TODO: choose level of assistance
        Pose2d pickupPose = drivetrain.getOffsetFuelPickupPose(fuel.get());
        // drivetrain.fieldOrientedDriveWhileAiming(duncan.getRequestedFieldOrientedVelocity(), pickupPose.getRotation());
        // drivetrain.fieldOrientedDriveOnALine(duncan.getRequestedFieldOrientedVelocity(), pickupPose);
        drivetrain.pidToPose(pickupPose, 1.0);
        });
    }

// AUTOS -------------------------------------------------------------------------------

public Command getAutonomousCommand() {
    return null;
}

    private Command trenchAutos() {
        try{
        // choose which paths to run
        String firstString, secondString;
        switch (autoChooser.getSelected()) {
            case "Deep":
                firstString = "DeepP1";
                secondString = "DeepP2";
            case "Shallow":
                firstString = "ShallowP1";
                secondString = "ShallowP2";
            case "DeepShallow":
                firstString = "DeepP1";
                secondString = "ShallowP2";
            case "ShallowDeep":
                firstString = "ShallowP1";
                secondString = "DeepP2";
            default:
                firstString = "DeepP1";
                secondString = "DeepP2";
        }

        PathPlannerPath firstPath = PathPlannerPath.fromPathFile(firstString);
        PathPlannerPath secondPath = PathPlannerPath.fromPathFile(secondString);

        // flip based on left or right
        double distFromLeftTrench = drivetrain.getPoseMeters().getTranslation().getDistance(FieldElement.TRENCH_LEFT.getLocation2d());
        double distFromRightTrench = drivetrain.getPoseMeters().getTranslation().getDistance(FieldElement.TRENCH_RIGHT.getLocation2d());

        if ( distFromRightTrench < distFromLeftTrench) {
            firstPath = firstPath.mirrorPath();
            secondPath = secondPath.mirrorPath();

        }

        return new SequentialCommandGroup(
            new ProxyCommand(AutoBuilder.followPath(firstPath)),
            Commands.waitSeconds(3),
            new ProxyCommand(aimAndShoot(() -> TurretCalculations.possibeTargets.hub, () -> false, () -> false).until(() -> (turret.getHoodAngleDeg() > 60))),
            new ProxyCommand(AutoBuilder.followPath(secondPath))
        );
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
    }
}
