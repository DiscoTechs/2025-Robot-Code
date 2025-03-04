// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterElevator;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterElevatorCommand extends Command {

  private final ShooterElevator shooterElevator;
  private final Joystick stick;

  /** Creates a new AlgaeCommand. */
  public ShooterElevatorCommand(ShooterElevator shooterElevator, Joystick stick) {

    this.shooterElevator = shooterElevator;
    this.stick = stick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterElevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterElevator.detectCoral()) {
      while(shooterElevator.detectCoral()) {
        shooterElevator.outtake();
        shooterElevator.stopElevator();
      }
      shooterElevator.stopShooter();
    }

    if (shooterElevator.detectCoral() || shooterElevator.reachedLimit()) { 
      shooterElevator.stopElevator();
    } else if (stick.getRawButton(Constants.ElavatorConstants.L1)) {
      shooterElevator.firstLevel();
    } else if (stick.getRawButton(Constants.ElavatorConstants.L2)) {
      shooterElevator.secondLevel();
    } else if (stick.getRawButton(Constants.ElavatorConstants.L3)) {
      shooterElevator.thirdLevel();
    } else if (stick.getRawButton(Constants.ElavatorConstants.L4)) {
      shooterElevator.fourthLevel();
    } else {
      shooterElevator.stopElevator();
    }

    //CORAL SHOOTER
    if (stick.getRawButton(Constants.CoralConstants.CORAL_INTAKE)) {
      shooterElevator.intake();
    } else if (stick.getRawButton(Constants.CoralConstants.CORAL_OUTTAKE)) {
      shooterElevator.outtake();
    } else {
      shooterElevator.stopShooter();
    }

    //manual control of elevator
    double speed = -stick.getRawAxis(Constants.ElavatorConstants.MANUAL_CONTROL_AXIS); //potnetially change sign from psotive to negative
    if (shooterElevator.detectCoral() || shooterElevator.reachedLimit()) { 
      shooterElevator.stopElevator();
    } else if(speed < 0 && (!shooterElevator.L1getSensorValue())) {
      shooterElevator.moveDown();
    } else if (speed > 0) {
      shooterElevator.moveUp();
    }
    else {
      shooterElevator.stopElevator();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
