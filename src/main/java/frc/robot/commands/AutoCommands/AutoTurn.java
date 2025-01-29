// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoTurn extends Command {

  private double target = 0.0;    // the passed parameter (degrees)
  private double setPoint = 0.0;  // The target translated to a relative heading
  private boolean atSetPoint = false;

  private SwerveSubsystem swerveSubsystem;
  private ChassisSpeeds chassisSpeeds;
  private ChassisSpeeds discreteSpeeds;
  private SwerveModuleState[] moduleStates;

  /** Creates a new AutoTurn. */
  public AutoTurn(SwerveSubsystem swerveSubsystem, double target) {

    this.target = target;
    this.setPoint = 0;
    this.swerveSubsystem = swerveSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    setPoint = swerveSubsystem.getHeading() + target;
    SmartDashboard.putNumber("Set Point", setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double speed = 0.8;

    // set the turn direction
    speed = (target < 0 ? speed : -speed);

    if (Math.abs(target - swerveSubsystem.getHeading()) < 20 ) {
      speed /= 2;
      atSetPoint = true;
    }

    if (Math.abs(target - swerveSubsystem.getHeading()) < 5 ) {
      speed = 0;
      atSetPoint = true;
    }
    
    chassisSpeeds = new ChassisSpeeds(0, 0, speed);
    discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);

    // 6. Output each module states to wheels
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
    return atSetPoint;
  }
}
