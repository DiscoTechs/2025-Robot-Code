// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // ------------ MODULE CONSTANTS ------------ //
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 7 / 150.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 1.00;

        public static final int gyroPort = 11;
    }

    // ------------ DRIVE CONSTANTS ------------ //
    public static final class DriveConstants {
        public static final double kTrackWidth = Units.inchesToMeters(21.75);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(21.75);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        // SPARK MAXES
        public static final double driveBaseRadius = Units.inchesToMeters(21.75 * Math.sqrt(2) / 2);

        // drive 40 amps
        public static final int kFrontLeftDriveMotorPort = 7;
        public static final int kBackLeftDriveMotorPort = 9;
        public static final int kFrontRightDriveMotorPort = 1;
        public static final int kBackRightDriveMotorPort = 14;

        // turning 20 amps
        public static final int kFrontLeftTurningMotorPort = 3;
        public static final int kBackLeftTurningMotorPort = 8;
        public static final int kFrontRightTurningMotorPort = 2;
        public static final int kBackRightTurningMotorPort = 17;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 2;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 1;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
        public static final int kBackRightDriveAbsoluteEncoderPort = 0;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final int kLeftFrontModule = 0;
        public static final int kRightFrontModule = 1;
        public static final int kLeftBackModule = 2;
        public static final int kRightBackModule = 3;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.383;//3.545944; //2.76; //2.75762;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 4.294;//1.175327; //1.91; //1.97222;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 6.150;//3.021143;// 0.49; //0.488692;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad =5.255; //2.095080; // 4.61; //4.64258;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.60248;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = 4.5; //4.5; //kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
               // kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    // ------------ AUTO CONSTANTS ------------ //
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = 
                            DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        // public static final double kPXController = 1.5;
        // public static final double kPYController = 1.5;
        // public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);

        public static final PPHolonomicDriveController holonomicDriveController = new PPHolonomicDriveController(
            new PIDConstants(0.05, 0.0, 0.0), // Translation PID constants
            new PIDConstants(0.025, 0.0, 0.0) // Rotation PID constants
        );
    }

    // ------------ OI CONSTANTS ------------ //
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int kButtonBoxPort = 2;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.1;

        public static int XBX_L_X = 0;
        public static int XBX_L_Y = 1;
        public static int XBX_L_TRIG = 2;
        public static int XBX_R_TRIG = 3;
        public static int XBX_R_X = 4;
        public static int XBX_R_Y = 5;

        public static int XBX_A = 1;
        public static int XBX_B = 2;
        public static int XBX_C = 3;
        public static int XBX_D = 4;
        public static int LEFT_BUMPER = 5;
        public static int RIGHT_BUMPER = 6;

    }

    // CONSTANTS BELOW ARE TEMPORARY AND NEED TO BE CHANGED TO MATCH JOYSTICK/BUTTON BOX

    // ------------ ALGAE CONSTANTS (CAN IDS AND RESPECTIVE BUTTONS TO CONTROL THEM) ------------ //
    public static final class AlgaeConstants {
        //ALGAE CAN IDS
        public static final int kLeftAlgaeEffectorMotorPort = 28; public static final int kRightAlgaeEffectorMotorPort = 39; //temporary CAN ID
            public static final int ALGAE_INTAKE = 8;
            public static final int ALGAE_OUTTAKE = 9;
        public static final int kAlgaeAngleMotorPort = 27; //temporary CAN ID
            public static final int ALGAE_ANGLE_UP = 10;
            public static final int ALGAE_ANGLE_DOWN = 12;
        
        public static final int kHighEncoderValue = 1;
        public static final int kLowEncoderValue = 0;

        public static final int kMaxEncoderValue = 2;
        public static final int kMinEncoderValue = -1;
    }

    // ------------ CORAL CONSTANTS (CAN IDS AND RESPECTIVE BUTTONS TO CONTROL THEM) ------------ //
    public static final class CoralConstants {
        //CORAL CAN IDS
        public static final int kLeftCoralEffectorMotorPort =  31; public static final int kRightCoralEffectorMotorPort =  41; //temporary CAN ID
            public static final int CORAL_INTAKE = 5;
            public static final int CORAL_OUTTAKE = 6;
            public static final int keepGoing = 7;
        public static final int kCoralPlateAngleMotorPort = 32; //final CAN ID
            public static final int CORAL_PLATE_ANGLE_UP = 1; //15
            public static final int CORAL_PLATE_ANGLE_DOWN = 2; //16
            public static final int CORAL_PLATE_ANGLE_DEFAULT = 3; //17

        //CORAL DELTA, DEFAULT, SPEED ENCODER VALUES FOR CORALPLATEANGLE
        public static final double kDelta = 0.01; //small value that can be adjusted
        public static final double kDefaultEncoder = 0.2; //default through-bore encoder value
        public static final double kAngleSpeed = 0.2; //default through-bore encoder value

        public static final double MIN_ENCODER_VALUE = 0;
        public static final double MAX_ENCODER_VALUE = 1;
        
        public static final double l1Speed = 0.3;
        public static final double l2speed = 0.5;
        public static final double l3speed = 0.5;
        public static final double l4speed = 0.3;

        public static final double defaultSpeed = 0.5;
    }

    // ------------ ELEVATOR CONSTANTS (CAN IDS AND RESPECTIVE BUTTONS TO CONTROL THEM) ------------ //
    public static final class ElavatorConstants {
        //ELEVATOR CAN IDS - NEED TO ADD MORE AS ELEVATOR HAS MORE MOTORS
        public static final int kRightElevatorMotorPort = 26;
        public static final int kLeftElevatorMotorPort = 29;
            public static final int L1 = 1;
            public static final int L2 = 2;
            public static final int L3 = 3;
            public static final int L4 = 4;
            public static final int MANUAL_CONTROL_AXIS = 1;
        
        //SENSOR PORT
        public static final int CORAL_SENSOR = 7;
        public static final int ELAVATOR_SENSOR_0 = 0;
        public static final int ELAVATOR_SENSOR_1 = 1;
        public static final int ELAVATOR_SENSOR_2 = 2;
        public static final int ELAVATOR_SENSOR_3 = 3;
        public static final int ELAVATOR_SENSOR_4 = 5;
        public static final int ELAVATOR_SENSOR_5 = 6;
    }

    // ------------ CLIMBER CONSTANTS (CAN IDS AND RESPECTIVE BUTTONS TO CONTROL THEM) ------------ //
    public static final class ClimberConstants {
        //CLIMBER CAN IDS
        public static final int kClimberMotorPort = 37;
            public static final int CLIMBER_UP = 13;
            public static final int CLIMBER_DOWN = 14;
    }




    // ------------ VISION CONSTANTS ------------ //
    public static final class VisionConstants {
        public static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE_MAP = new InterpolatingDoubleTreeMap();

        static { DISTANCE_TO_ANGLE_MAP.put(30.0, .75); }
        public static final InterpolatingDoubleTreeMap DISTANCE_TO_SHOOT_MAP = new InterpolatingDoubleTreeMap();
        static { DISTANCE_TO_SHOOT_MAP.put(30.0, 4000.0); }
    }
}