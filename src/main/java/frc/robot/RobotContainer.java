// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.HumanDriver;
import frc.robot.subsystems.LedsCANdle;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.GyroIOPigeon;
import frc.robot.subsystems.drivetrain.GyroIOSim;
import frc.robot.subsystems.drivetrain.SwerveModuleIOKraken;
import frc.robot.subsystems.drivetrain.SwerveModuleIOSim;

public class RobotContainer {

    public final Drivetrain drivetrain;

    // private final SendableChooser<String> autoChooser;
    
    protected final HumanDriver duncan = new HumanDriver(0);
    final CommandXboxController duncanController;
    public final LedsCANdle ledsCANdle;

    public RobotContainer() {
        /**** INITIALIZE SUBSYSTEMS ****/
        if (RobotBase.isReal()) { //TODO put this back to not !
            // NOODLE OFFSETS: FL -0.184814453125, FR 0.044677734375, BL -0.3349609375, BR 0.088134765625 
            drivetrain = new Drivetrain( 
                new GyroIOPigeon(),
                new SwerveModuleIOKraken(4, 5, 0.062012, 6, "FL", false), 
                new SwerveModuleIOKraken(1, 2, 0.450928, 3, "FR", true),
                new SwerveModuleIOKraken(7, 8,0.080811, 9, "BL", false),
                new SwerveModuleIOKraken(10, 11,  0.160889, 12, "BR", true) 
            );
        } else {
            drivetrain = new Drivetrain(
                new GyroIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){},
                new SwerveModuleIOSim(){}
            );
        }

        // canLedsCounter = new LedsCANdle(45, 60);

        // drivetrain.setFocus(FieldElement.HUB);
        ledsCANdle = new LedsCANdle(13, 275);
        duncanController = duncan.getXboxController();
        configureBindings();
        setDefaultCommands();

    }

    private void configureBindings() {

        duncanController.y().onTrue(reSeedRobotPose());
        duncanController.start().onTrue(Commands.runOnce(drivetrain::setRobotFacingForward));
    }

    public void setDefaultCommands() {
        drivetrain.setDefaultCommand(new ConditionalCommand(driverFullyControlDrivetrain().withName("driveDefualtCommand"),
            drivetrain.run(() ->drivetrain.playMusic("song"))
                .finallyDo(() -> drivetrain.stopMusic()), 
                () -> DriverStation.isEnabled()));
        ledsCANdle.setDefaultCommand(ledsCANdle.heartbeatCommand().ignoringDisable(true));
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
