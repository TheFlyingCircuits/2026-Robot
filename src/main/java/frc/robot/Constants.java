// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final static boolean atCompetition = false;

    public final class UniversalConstants {
        public final static double gravityMetersPerSecondSquared = 9.81;
        public final static double defaultPeriodSeconds = 0.02;

        public final static String canivoreName = "CTRENetwork";

        public enum Direction {
            left,
            right
        }

        public static final double frameWidthMeters = Units.inchesToMeters(27);

        public static final double bumperWidthMeters = Units.inchesToMeters(27 + 7);

    }

    public final static class ControllerConstants {
        public static final double controllerDeadzone = 0.075;
        public static final double maxThrottle = 1.0;
    }

    public final static class DrivetrainConstants {
        // KINEMATICS CONSTANTS

        /**
         * Distance between the center point of the left wheels and the center point of the right wheels.
         */
        public static final double trackwidthMeters = Units.inchesToMeters(21.75);
        /**
         * Distance between the center point of the front wheels and the center point of the back wheels.
         */
        public static final double wheelbaseMeters = Units.inchesToMeters(21.75);
        /**
         * Distance from the center of the robot to each swerve module.
         */
        public static final double drivetrainRadiusMeters = Math.hypot(wheelbaseMeters / 2.0, trackwidthMeters / 2.0); //0.4177


        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
            new Translation2d(wheelbaseMeters / 2.0, -trackwidthMeters / 2.0),
            new Translation2d(-wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
            new Translation2d(-wheelbaseMeters / 2.0, -trackwidthMeters / 2.0)
        );

        public static final double frameWidthMeters = Units.inchesToMeters(27);

        public static final double bumperWidthMeters = Units.inchesToMeters(27 + 7);
        public static final double halfBumperWidthMeters = bumperWidthMeters / 2.0;



        /**
         * The maximum possible velocity of the robot in meters per second.
         * <br>
         * This is a measure of how fast the robot will be able to drive in a straight line, based off of the empirical free speed of the drive Krakens.
         */
        // public static final double maxAchievableVelocityMetersPerSecond = 1.0;
        public static final double krakenFreeSpeedRPM = 5800;
        public static final double krakenFreeSpeedRotationsPerSecond = krakenFreeSpeedRPM / 60.;
        public static final double maxAchievableVelocityMetersPerSecond = krakenFreeSpeedRotationsPerSecond *
            (1.0/SwerveModuleConstants.driveGearReduction) * SwerveModuleConstants.wheelCircumferenceMeters; // ~5.23 using a theoretical wheel radius of 2 inches m/s
                                                                                                       // ~5.06 when adding 1/16 of an inch of wheel sink into the carpet.
                                                                                                       // ~5.10 using an emperical measurement of wheel radius on fresh wheels.
                                                                                                       // Actual top speed based on testing is ~4.7 m/s
                                                                                                       // (calculating top speed using kv yeilds [12 / 2.42] ~ 4.96 m/s,
                                                                                                       //  but I don't think we can actually achieve this because 
                                                                                                       //  the battery voltage will likely drop below 12 when all drive motors are running)
                                                                                                       // To give ourselves a little breathing room, we use a max speed of 4.5 m/s in auto.

        /**
         * This is the max desired speed that will be achievable in teleop.
         * <br>
         * If the controller joystick is maxed in one direction, it will drive at this speed.
         * <br>
         * This value will be less than or equal to the maxAchievableVelocityMetersPerSecond, depending on driver preference.
         */
        public static final double maxDesiredTeleopVelocityMetersPerSecond = maxAchievableVelocityMetersPerSecond; 

        /**
         * The maximum achievable angular velocity of the robot in radians per second.
         * <br>
         * This is a measure of how fast the robot can rotate in place, based off of maxAchievableVelocityMetersPerSecond.
         */

        public static final double maxAchievableAngularVelocityRadiansPerSecond = maxAchievableVelocityMetersPerSecond / drivetrainRadiusMeters; // Theoretical ~1.93 rotations per second
                                                                                                                                                 // using 4.7 m/s for max linear speed yeilds ~1.79 rotations per second
                                                                                                                                                 // using 4.5 m/s for max linear speed yeilds ~1.71 rotations per second
                                                                                                                                                 // we use 1.0 rotations per second in auto to be extra conservative
                                                                                                                                                 // because any time you're rotating, you're taking away from your translational speed.

        /**
         * This is the max desired angular velocity that will be achievable in teleop.
         * <br>
         * If the controller rotation joystick is maxed in one direction, it will rotate at this speed.
         * <br>
         * This value will be tuned based off of driver preference.
         */
        public static final double maxDesiredTeleopAngularVelocityRadiansPerSecond = Units.rotationsToRadians(0.85);


        public static final PathConstraints pathfindingConstraints = new PathConstraints(
                1.0, 1.0,
                Units.degreesToRadians(360), Units.degreesToRadians(360));
    }


    public final static class SwerveModuleConstants {
        /** Rotations of the drive wheel per rotations of the drive motor. */
        public static final double driveGearReduction = (50.0 / 15.0) * (17.0 / 27.0) * (45.0 / 15.0);

        /** Rotations of the steering column per rotations of the angle motor. */
        public static final double steerGearReduction = (50.0 / 14.0) * (60.0 / 10.0);

        // The wheels have a 2 inch radius, but sink into the capet about (1/16) of an inch.
        // As an estimate, the wheel radius is Units.inchesToMeters(2.-1./16.), or 0.0492m
        // public static final double wheelRadiusMeters = 0.04946; //use MeasureWheelDiameter for this!
        public static final double wheelRadiusMeters = Units.inchesToMeters(4)/2.0; 
        public static final double wheelCircumferenceMeters = 2 * Math.PI * wheelRadiusMeters; // ~0.31
    }


    public final static class GyroConstants {
        public static final int pigeonID = 0;


        //Follow the mount calibration process in Phoenix Tuner to determine these
        public static final double mountPoseYawDegrees = 92.07196807861328;
        public static final double mountPosePitchDegrees = -0.24960607290267944;
        public static final double mountPoseRollDegrees = -0.591957151889801;
    }

    public final static class TurretConstants {
        public static final double aimerCANcoderOffset = 0.0; // get from tuner

        public static final double turretMaxAngle = 180; // need to change for real
        public static final double turretMinAngle = -180;

        public static final double maxHoodAngle = 85.0;
        public static final double minHoodAngle = 45.7;


        public static final int aimerKrakenID = 99;
        public static final int aimerCANcoderID = 99;
        public static final int hoodKrakenID = 99;
        public static final int frontWheelKrakenID = 99;
        public static final int frontWheelFollowerKrakenID = 99;
        public static final int hoodWheelKrakenID = 99;

        // 27:160 - ctre wants a >1 number for reductions
        public static final double aimerKrakenToTurretRotationsGearRatio = 5.92592592593;// this is final I think and its cancoder to turret
        // 2:1
        public static final double aimerKrakenRotorToCANcoderGearRatio = 2.0;
        
        // TODO: get real ratios and set gains to 0
        public static final double hoodKrakenToTurretRotationsGearRatio = 3.0;

        public static final double mainWheelKrakenToTurretRotationsGearRatio = 3.0;
        public static final double hoodWheelKrakenToTurretRotationsGearRatio = 3.0;

        public static final double hoodDefaultAngleDegrees = 80.0;

        public static final double mainFlywheelDiameterMeters = Units.inchesToMeters(3);
        public static final double hoodFlywheelDiameterMeters = Units.inchesToMeters(2);
    }

    public final static class IndexerConstants {
        public static final int bigSpinnerID = 17;
        public static final int sideKickerID = 15;
        public static final int kickerID = 16;

        public static final double bigSpinnerGearRatio = 2.0; // positive is gear reduction
        public static final double sideKickerGearRatio = 2.0;
        public static final double kickerGearRatio = 28.0;
    }

    public final static class IntakeConstants {

        public static final int pivotNeoID = 99;
        public static final int rollerTopKrakenID = 99;
        public static final int rollerBottomKrakenID = 99;
        public static final double kP = 0.0;
        public static final double kS = 0.0;

        public static final double intakeStartDegrees = 45.0; 
    }

    public final static class VisionConstants {
        //Camera, IP, hostname
        //http://10.17.87.12:5800 this is for front/BW2 and fuel/C2     
        //http://10.17.87.11:5800/ this is for left/BW3, right/BW4, and back/BW1                                           

        public final static Transform3d robotToFront = new Transform3d(
            new Translation3d(Units.inchesToMeters(7.248), Units.inchesToMeters(11.275), Units.inchesToMeters(7.281)),
            new Rotation3d(0, -Math.toRadians(8), -Math.toRadians(17.772))
        );

        public final static Transform3d robotToLeft = new Transform3d(
            new Translation3d(Units.inchesToMeters(7.248), -Units.inchesToMeters(11.275), Units.inchesToMeters(7.281)),
            new Rotation3d(0, -Math.toRadians(8), Math.toRadians(17.772))
        );

        public final static Transform3d robotToRight = new Transform3d(
            new Translation3d(-0.184, Units.inchesToMeters(11.275), Units.inchesToMeters(7.281)),
            new Rotation3d(0, Math.toRadians(-8), Math.toRadians(17.772-180))
        );

        public final static Transform3d robotToBack = new Transform3d(
            new Translation3d(-Units.inchesToMeters(7.248), -Units.inchesToMeters(11.275), Units.inchesToMeters(7.281)),
            new Rotation3d(0, Math.toRadians(-8), Math.toRadians(-17.772+180))
        );

        public final static Transform3d robotToFuelCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(-9.75), Units.inchesToMeters(5.5), Units.inchesToMeters(26.)),
            new Rotation3d(0, Math.toRadians(19), Math.toRadians(-12))
        );

        public final static String[] tagCameraNames = {
            "front",
            "left",
            "right",
            "back"
        };

        public final static Transform3d[] tagCameraTransforms = {
            robotToFront,
            robotToLeft,
            robotToRight,
            robotToBack
        };

    }

    public final static class LEDConstants {
        public final static int ledPWMPort = 0;

        //total number of leds
        public final static int ledsPerStrip = 60;

        public final static double stripLengthMeters = 1.0;

        public final static double ledsPerMeter = (1.0 * ledsPerStrip) / stripLengthMeters;

        public final static double metersPerLed = 1/ledsPerMeter;

        /**
         * Hues for specific colors
         * Values use the openCV convention where hue ranges from [0, 180)
         */
        public final static class Hues {

            public final static int orangeSignalLight = 4;
            public final static int blueBumpers = 114;
            public final static int redBumpers = 0;
            public final static int redTrafficLight = 0;//0;
            public final static int greenTrafficLight = 40;//60;
            public final static int betweenBlueAndRed = 150; // a purple/pink that's between blue and red.

        }
    }

}
