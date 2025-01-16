// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

enum Module {
  FRONT_LEFT,
  FRONT_RIGHT,
  BACK_LEFT,
  BACK_RIGHT
}

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDriveOdometry odometry;
  public final Pigeon2 gyro;
  ChassisSpeeds chassisSpeeds;

  private SwerveModule[] modules = new SwerveModule[4];

  public SwerveSubsystem() {
    gyro = new Pigeon2(0);
    odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, gyro.getRotation2d(), getModulePositions());

    // Initialize the modules
    modules[Module.FRONT_LEFT.ordinal()] = new SwerveModule(0, 0, false, false, 0, 0, false);

    new Thread(() -> {
      try {
        Thread.sleep(1000);
        gyro.reset();
      } catch (Exception e) {}
    }).start();
  }

  public double getHeading() {
    return Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360);
  }

  public void resetGyro() {
    gyro.reset();
  }

  public SwerveModulePosition getModulePosition(m Module) {
    SwerveModule module = modules[m];
    return new SwerveModulePosition(test.getDrivePosition(), gyro.getRotation2d());
  }

  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void stop() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].stop();
    }
  }
}


/*
    private SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

       private SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
        
      private SwerveModule  backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

       private SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    public SwerveModulePosition getFrontLeftModulePosition() {
        return new SwerveModulePosition(frontLeft.getDrivePosition(), gyro.getRotation2d());
    }

    public SwerveModulePosition getFrontRightModulePosition() {
        return new SwerveModulePosition(frontRight.getDrivePosition(), gyro.getRotation2d());
    }

  public SwerveModulePosition getBackLeftModulePosition() {
    return new SwerveModulePosition(backLeft.getDrivePosition(), gyro.getRotation2d());
  }

  public SwerveModulePosition getBackRightModulePosition() {
    return new SwerveModulePosition(backRight.getDrivePosition(), gyro.getRotation2d());
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      getFrontLeftModulePosition(),
      getFrontRightModulePosition(),
      getBackLeftModulePosition(),
      getBackRightModulePosition()
    };
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }
  
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, odometry.getPoseMeters().getRotation()));
    }

    public ChassisSpeeds getSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), odometry.getPoseMeters());
    }

    @Override
    public void periodic() {
      
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[1]); //1
        frontRight.setDesiredState(desiredStates[0]); //041
        backLeft.setDesiredState(desiredStates[3]); //3
        backRight.setDesiredState(desiredStates[2]); //2
    }
}
*/