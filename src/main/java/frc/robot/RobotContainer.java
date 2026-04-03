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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
import frc.robot.Constants.TurretConstants;
import frc.robot.PlayingField.FieldElement;
import frc.robot.commands.AimAndShoot;
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
        if (RobotBase.isReal()) { //TODO put this back to not !
            // NOODLE OFFSETS: FL -0.184814453125, FR 0.044677734375, BL -0.3349609375, BR 0.088134765625 
            drivetrain = new Drivetrain( 
                new GyroIOPigeon(),
                new SwerveModuleIOKraken(1, 2, -0.260498, 1, "FL", true), 
                new SwerveModuleIOKraken(3, 4, 0.429199, 2, "FR", false),
                new SwerveModuleIOKraken(5, 6, -0.033203, 3, "BL", true),
                new SwerveModuleIOKraken(7, 8,  0.098389, 4, "BR", false) 
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
        // canLedsCounter = new LedsCANdle(45, 60);

        // drivetrain.setFocus(FieldElement.HUB);

        FlyingCircuitUtils.putNumberOnDashboard("proportion", 1.89);
        FlyingCircuitUtils.putNumberOnDashboard("intercept", - 5.93);

        // FlyingCircuitUtils.getNumberFromDashboard("frontWheelVolts", 0.0);

        // FlyingCircuitUtils.putNumberOnDashboard("aimerTargetVolts", 0.0);

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
        new EventTrigger("intake").onTrue(new ProxyCommand(intake.inAutoIntakeCommand()));
        // new EventTrigger("aim").whileTrue(aimAndShoot(() -> TurretCalculations.possibeTargets.hub, () -> false, () -> true));
        new EventTrigger("aim").whileTrue(new ProxyCommand(new AimAndShoot(turret, indexer, () -> TurretCalculations.getTurretTranslation(drivetrain.getPoseMeters().getTranslation()), 
        () -> drivetrain.getFieldOrientedVelocity(), () -> false, true, drivetrain)
        .alongWith(intake.inAutoIntakeCommand())));

        new EventTrigger("shoot").onTrue(new ProxyCommand(aimAndShoot(() -> true, () -> true)
        .alongWith(intake.intakeDefualtAndIntakeCommand())));

        configureBindings();
        setDefaultCommands();
    }

    private void configureBindings() {
        // duncanController.a().whileTrue(new ShootWithParams(turret, indexer, ()->0.0, 
        //     () -> FlyingCircuitUtils.getNumberFromDashboard("reqHoodAngle", 0.0), 
        //     () -> FlyingCircuitUtils.getNumberFromDashboard("reqMPS", 0.0)));

        // duncanController.a().whileTrue(turret.setAllVoltsCommand(()->0.0,()->0.0,
        //     () -> FlyingCircuitUtils.getNumberFromDashboard("frontWheelVolts", 0.0), ()->0.0));

        duncanController.rightStick().onTrue(aimAndShoot(() -> false, () -> true));
        duncanController.leftStick().onTrue(aimAndShoot(() -> false, () -> true));

        duncanController.rightBumper().onTrue(aimAndShoot(() -> true, () -> true).alongWith(intake.intakeDefualtAndIntakeCommand()));
        // .alongWith(canLedsCounter.playFireNoteAnimationCommand()));

        // duncanController.rightBumper().whileTrue(new ShootWithParams(turret, indexer, () -> 0.0, () -> 60.0 
        // , () -> FlyingCircuitUtils.getNumberFromDashboard("targetMPS", 0.0)));

        duncanController.leftTrigger().whileTrue(intake.intakeDefualtAndIntakeCommand()
            .alongWith(aimAndShoot(() -> false, () -> true))).whileFalse(
                aimAndShoot(() -> false, () -> true)
            );// also aims

        duncanController.y().onTrue(reSeedRobotPose());
        duncanController.start().onTrue(Commands.runOnce(drivetrain::setRobotFacingForward));

        // reset everything
        duncanController.x().onTrue(Commands.runOnce(() -> {
            CommandScheduler.getInstance().cancelAll();
        }));

        // duncanController.a().whileTrue(intake.intakeDownCommand());
        duncanController.povUp().whileTrue(intake.setAllVoltsCommand(()->0.0, ()->0.0, () ->7.0));
        duncanController.povDown().whileTrue(intake.setAllVoltsCommand(()->0.0, ()->0.0, () ->-7.0));
        duncanController.povRight().whileTrue(intake.intakeDownCommand().until(() -> intake.isIntakeDown()).andThen(intake.intakeDefualtAndIntakeCommand()));
        duncanController.povLeft().whileTrue(indexer.indexFuelCommand());

        duncanController.rightTrigger().whileTrue(intake.reverseIntakeCommand().alongWith(indexer.reverseIndexerCommand()));
    }

    public void setDefaultCommands() {
        drivetrain.setDefaultCommand(driverFullyControlDrivetrain().withName("driveDefualtCommand"));
        turret.setDefaultCommand(turret.turretStopDoingStuffCommand());
        indexer.setDefaultCommand(indexer.stopIndexingCommand());
        intake.setDefaultCommand(intake.noVoltageCommand());
        // canLedsCounter.setDefaultCommand(canLedsCounter.solidColorCommand(Color.fromHSV(canLedsCounter.getAllianceHue(), 255, 255)).ignoringDisable(true));
    }

    public void periodic() {
        // Log turret pose, isn't part of turret because it needs the drivetrain too.
        Translation3d turretLocation = TurretCalculations.getTurretTranslation(drivetrain.getPoseMeters().getTranslation());
        Rotation2d turretYaw_robotCoords = Rotation2d.fromDegrees(turret.getAimerAngleDeg_robotCoords());
        Rotation2d turretYaw_fieldCoords = turretYaw_robotCoords.rotateBy(drivetrain.getPoseMeters().getRotation());

        // pitch viz comes from hood angle.
        // negate to match wpilib convention of looking up being negative pitch.
        double turretPitchRadians = -Units.degreesToRadians(turret.getHoodAngleDeg());

        Rotation3d turretOrientation = new Rotation3d(0, turretPitchRadians, turretYaw_fieldCoords.getRadians());
        Logger.recordOutput("turret/poseOnField", new Pose3d(turretLocation, turretOrientation));
    }

    private Command aimAndShoot(Supplier<Boolean> driverReadyToShoot, Supplier<Boolean> needsReqs) {
        return new AimAndShoot(turret, indexer, () -> TurretCalculations.getTurretTranslation(drivetrain.getPoseMeters().getTranslation()), 
        () -> drivetrain.getFieldOrientedVelocity(), driverReadyToShoot, needsReqs.get(), drivetrain);
    }

    private Command aimAndShootManual(Supplier<Boolean> driverReadyToShoot,
    Supplier<Translation3d> manualTurretPose) {
        return new AimAndShoot(turret, indexer, manualTurretPose, 
        () -> drivetrain.getFieldOrientedVelocity(), driverReadyToShoot, true, drivetrain);
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

    // private Command reSeedRobotPoseOnTimeInterval(double timeIntervalSec) {return new ConditionalCommand(
    //     new InstantCommand(() -> trustCamerasTimer.reset()).alongWith(new InstantCommand(() -> System.out.println("lllllllllllllllllllllllllllllllllll"))),
    //     Commands.run(() -> {
    //     drivetrain.fullyTrustVisionNextPoseUpdate();
    //     drivetrain.allowTeleportsNextPoseUpdate();
    //     // System.out.println("trusting Vision");
    // }).until(drivetrain::seesAcceptableTag).ignoringDisable(true)
    // , () -> (trustCamerasTimer.get() < timeIntervalSec));}


    private Command driveTowardsFuelTeleop() { return drivetrain.run(() -> {
        Optional<Translation3d> fuel = drivetrain.getClosestCluster();
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
        Translation2d robotToFuel = fuel.get().toTranslation2d().minus(drivetrain.getPoseMeters().getTranslation());
        Pose2d pickupPose = new Pose2d(fuel.get().toTranslation2d(), robotToFuel.getAngle());
        // drivetrain.fieldOrientedDriveWhileAiming(duncan.getRequestedFieldOrientedVelocity(), pickupPose.getRotation());
        // drivetrain.fieldOrientedDriveOnALine(duncan.getRequestedFieldOrientedVelocity(), pickupPose);
        drivetrain.pidToPose(pickupPose, 1.0);
        });
    }

    // AUTOS -------------------------------------------------------------------------------

    public Command getAutonomousCommand() {
        return agressiveDipAuto();
        // return trenchAutos();
    }

    private Command passingLeftAuto() {
        try {
            PathPlannerPath firstPath = PathPlannerPath.fromPathFile("Passing Left");
            // flip based on left or right
            double distFromLeftTrench = drivetrain.getPoseMeters().getTranslation().getDistance(FieldElement.TRENCH_LEFT.getLocation2d());
            double distFromRightTrench = drivetrain.getPoseMeters().getTranslation().getDistance(FieldElement.TRENCH_RIGHT.getLocation2d());

            if (distFromRightTrench < distFromLeftTrench) {
                firstPath = firstPath.mirrorPath();
            }
                return new SequentialCommandGroup(
                new ProxyCommand(AutoBuilder.followPath(firstPath)),
                Commands.waitSeconds(6));
            } catch (Exception e) {
                DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
                return Commands.none();
        }
    }

    private Command agressiveDipAuto() {
        try{
        PathPlannerPath firstPath = PathPlannerPath.fromPathFile("Agressive Dip P1");
        PathPlannerPath secondPath = PathPlannerPath.fromPathFile("Agressive Dip P2 MID");

        // flip based on left or right
        double distFromLeftTrench = drivetrain.getPoseMeters().getTranslation().getDistance(FieldElement.TRENCH_LEFT.getLocation2d());
        double distFromRightTrench = drivetrain.getPoseMeters().getTranslation().getDistance(FieldElement.TRENCH_RIGHT.getLocation2d());

        if ( distFromRightTrench < distFromLeftTrench) {
            firstPath = firstPath.mirrorPath();
            secondPath = secondPath.mirrorPath();
        }

        return new SequentialCommandGroup(

            new ProxyCommand(AutoBuilder.followPath(firstPath)),
            Commands.waitSeconds(4),
            new ProxyCommand(aimAndShoot(() -> false, () -> false).until(() -> (turret.getHoodAngleDeg() > TurretConstants.maxHoodAngle-10.0))),
            new ProxyCommand(AutoBuilder.followPath(secondPath)),
            Commands.waitSeconds(7)
        );
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
    }

    private Command trenchAutos() {
        try{
        // choose which paths to run
        String firstString, secondString;
        if (autoChooser.getSelected().equals("Deep")) {
            firstString = "DeepP1";
            secondString = "DeepP2";
        } else if (autoChooser.getSelected().equals("Shallow")) {
            firstString = "ShallowP1";
            secondString = "ShallowP2";
        } else if (autoChooser.getSelected().equals("DeepShallow")) {
            firstString = "DeepP1";
            secondString = "ShallowP2";
        } else if (autoChooser.getSelected().equals("ShallowDeep")) {
            firstString = "ShallowP1";
            secondString = "DeepP2";
        } else {
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
            new ProxyCommand(intake.intakeDownCommand().until(() -> intake.isIntakeDown())),
            new ProxyCommand(AutoBuilder.followPath(firstPath)),
            Commands.waitSeconds(3),
            new ProxyCommand(aimAndShoot(() -> false, () -> false).until(() -> (turret.getHoodAngleDeg() > TurretConstants.maxHoodAngle-10.0))),
            new ProxyCommand(AutoBuilder.followPath(secondPath)),
            Commands.waitSeconds(5)
        );
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
    }
}
