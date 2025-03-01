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
    //if the button is pressed, then it will move motor so that it is at default value
    if (stick.getRawButton(Constants.CoralConstants.CORAL_PLATE_ANGLE_DEFAULT)) {
        if (coralPlateAngle.getEncoder() < Constants.CoralConstants.kDefaultEncoder) { //if below default value (current zero), then will angle up
            coralPlateAngle.angleUpToDefault();
        }
        else if (coralPlateAngle.getEncoder() > Constants.CoralConstants.kDefaultEncoder) { //if above default value (current zero), then will angle down
            coralPlateAngle.angleDownToDefault();
        }
        else {
            coralPlateAngle.stopAngle();
        }
    } else if(stick.getRawButton(Constants.CoralConstants.CORAL_PLATE_ANGLE_UP)) { //if the button is pressed, then the plate will angle up. This is used to get coral unstuck.
      coralPlateAngle.angleUp();
    } else if (stick.getRawButton(Constants.CoralConstants.CORAL_PLATE_ANGLE_DOWN)) { //if the button is pressed, then the plate will angle down. This is used to get coral unstuck.
        coralPlateAngle.angleDown();
    } else { //the code under this else statement is what the plate angler will be doing most of the time: adjusting a bit up and down to counter gravity and robot movement to stay at the same default angle
        if (coralPlateAngle.getEncoder() < Constants.CoralConstants.kDefaultEncoder) { //if below default value (current zero), then will angle up
            coralPlateAngle.angleUpToDefault();
        }
        else if (coralPlateAngle.getEncoder() > Constants.CoralConstants.kDefaultEncoder) { //if above default value (current zero), then will angle down
            coralPlateAngle.angleDownToDefault();
        }
        else {
            coralPlateAngle.stopAngle();
        }
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
