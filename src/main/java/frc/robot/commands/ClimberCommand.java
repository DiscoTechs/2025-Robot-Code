// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Joystick;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberCommand extends Command {

  private final Climber climber;
  private final Joystick stick;

  /** Creates a new AlgaeCommand. */
  public ClimberCommand(Climber climber, Joystick stick) {

    this.climber = climber;
    this.stick = stick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double spd = -stick.getRawAxis(5);
    //System.out.println(spd);
    if (spd > .1) {
      climber.moveClimber(0.6); //
    } else if (spd < -.1) {
      climber.moveClimber(-0.6);
    } else {
      climber.moveClimber(0);
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
