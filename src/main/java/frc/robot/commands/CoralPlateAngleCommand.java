// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPlateAngle;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralPlateAngleCommand extends Command {

  private final CoralPlateAngle coralPlateAngle;
  private final Joystick stick;

  /** Creates a new CoralCommand. */
  public CoralPlateAngleCommand(CoralPlateAngle coralPlateAngle, Joystick stick) {

    this.coralPlateAngle = coralPlateAngle;
    this.stick = stick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralPlateAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(stick.getRawButton(Constants.CoralConstants.CORAL_PLATE_ANGLE_UP)) {
      coralPlateAngle.angleUp();
    } else if (stick.getRawButton(Constants.CoralConstants.CORAL_PLATE_ANGLE_DOWN)) {
        coralPlateAngle.angleDown();
    } else {
        coralPlateAngle.stopAngle();
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
