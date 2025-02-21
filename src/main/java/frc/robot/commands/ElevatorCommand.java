// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeEffector;
import frc.robot.subsystems.ElevatorEffector;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {

  private final ElevatorEffector elevatorEffector;
  private final Joystick stick;

  /** Creates a new AlgaeCommand. */
  public ElevatorCommand(ElevatorEffector elevatorEffector, Joystick stick) {

    this.elevatorEffector = elevatorEffector;
    this.stick = stick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stick.getRawButton(Constants.ElavatorConstants.L1)) {
      elevatorEffector.firstLevel();
    } else if (stick.getRawButton(Constants.ElavatorConstants.L2)) {
      elevatorEffector.secondLevel();
    } else if (stick.getRawButton(Constants.ElavatorConstants.L3)) {
      elevatorEffector.thirdLevel();
    } else if (stick.getRawButton(Constants.ElavatorConstants.L4)) {
      elevatorEffector.fourthLevel();
    } else {
      elevatorEffector.stop();
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
