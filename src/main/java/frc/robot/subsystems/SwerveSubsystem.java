// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;


// enum Module {
//   FRONT_LEFT,
//   FRONT_RIGHT,
//   BACK_LEFT,
//   BACK_RIGHT
// }

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDriveOdometry odometry;
  public final Pigeon2 gyro;
  ChassisSpeeds chassisSpeeds;

  // private SwerveModule[] modules = new SwerveModule[4];
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

  private SwerveModule backLeft = new SwerveModule(
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

  public SwerveSubsystem() {
    gyro = new Pigeon2(0);
    odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, gyro.getRotation2d(), getModulePositions());

    // // Initialize the modules
    // modules[Module.FRONT_LEFT.ordinal()] = new SwerveModule(0, 0, false, false, 0, 0, false);

    new Thread(() -> {
      try {
        Thread.sleep(1000);
        gyro.reset();
      } catch (Exception e) {
      }
    }).start();
  }

  public double getHeading() {
    return -Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360);
  }

  public boolean decideIfTurnLeft(double targetAngle) {
    double a = getHeading(); //being straight in field relative should be zero → print somewhere else too
    double t = targetAngle;
    double leftTurn = t-a;
    if (leftTurn < 0) {
      leftTurn += 360;
    }
    return (leftTurn < 180);
  }
  

  public void resetGyro() {
    gyro.reset();
  }

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

  /*public void printAbsoluteEncoder() {
    System.out.println("Absolute Encoder Left Front: " + frontLeft.getAbsoluteEncoderDeg());
    System.out.println("Absolute Encoder Right Front: " + frontRight.getAbsoluteEncoderDeg());
    System.out.println("Absolute Encoder Left Back" + backLeft.getAbsoluteEncoderDeg());
    System.out.println("Absolute Encoder Right Back" + backRight.getAbsoluteEncoderDeg());
  }*/

  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      getFrontLeftModulePosition(),
      getFrontRightModulePosition(),
      getBackLeftModulePosition(),
      getBackRightModulePosition()
    };
  }

  @Override
  public void periodic() {
       // System.out.println("SWERVE PERIODIC " + (int)(System.currentTimeMillis() / 1000 - 17413019) );
      odometry.update(getRotation2d(), getModulePositions());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Absolute Encoder Left Front", -frontLeft.getAbsoluteEncoderDeg()); //front left
        SmartDashboard.putNumber("Absolute Encoder Right Front", -frontRight.getAbsoluteEncoderDeg()); //front right
        SmartDashboard.putNumber("Absolute Encoder Left Back", -backLeft.getAbsoluteEncoderDeg()); //back left
        SmartDashboard.putNumber("Absolute Encoder Right Back", -backRight.getAbsoluteEncoderDeg()); //back right
        SmartDashboard.putString("Robot Location", getPose2d().getTranslation().toString());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
    DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontRight.setDesiredState(desiredStates[0]); //041
    frontLeft.setDesiredState(desiredStates[1]); //1
    backLeft.setDesiredState(desiredStates[3]); //3
    backRight.setDesiredState(desiredStates[2]); //2

    
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getModulePositions(), pose);
  }
    
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds,
    0.02);
    
    SwerveModuleState[] targetStates =
    DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }
    
  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds,
    odometry.getPoseMeters().getRotation()));
  }
    
  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
    frontRight.getState(), backLeft.getState(), backRight.getState());
  }
    
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(gyro.getRotation2d(), getModulePositions(),
    odometry.getPoseMeters());
  }

  public void stop() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }
}