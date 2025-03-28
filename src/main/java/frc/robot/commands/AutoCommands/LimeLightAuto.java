// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LimeLightAuto extends Command {

  private SwerveSubsystem swerveSubsystem;
  private ChassisSpeeds chassisSpeeds;
  private ChassisSpeeds discreteSpeeds;
  private SwerveModuleState[] moduleStates;
  
  private double xSpeed, ySpeed, turningSpeed;
  private int pipelineID = 0;
  
  /** Creates a new LimeLightAuto. */
  public LimeLightAuto(SwerveSubsystem swerveSubsystem, int pipeline) {
    this.swerveSubsystem = swerveSubsystem;
    this.pipelineID = pipeline;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    chassisSpeeds = limelightFollow();

    discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSpeeds = new ChassisSpeeds(0, 0, 0);
    moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    swerveSubsystem.setModuleStates(moduleStates); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public ChassisSpeeds limelightFollow() {

        //LimelightHelpers.setPipelineIndex("limelight", 1);

        double tx = LimelightHelpers.getTX("limelight");
        double ta = LimelightHelpers.getTA("limelight");

        double[] botPose = LimelightHelpers.getBotPose_TargetSpace("limelight");

        // Angle of the april tag
        double offsetAngle = botPose[4];

        // XSPEED
        xSpeed = 1.0 - ta / 100 * 1.1;  //110% of proportional value

        // YSPEED - slide left/right based on offset
        if(Math.abs(offsetAngle) > 5.0) {
            ySpeed = -(offsetAngle / 90) * 1.5; // 150% of proportional speed
        } else {
            ySpeed = 0;
        }

        // TURNING SPEED
        if (Math.abs(tx) < 1) {tx = 0;} // lame deadband code
        turningSpeed = -tx * 0.06;      // trial and error, kP

       return new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
   }
}
