// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeEffector;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeEffectorCommand extends Command {

  private final AlgaeEffector algaeEffector;
  private final Joystick stick;

  /** Creates a new AlgaeCommand. */
  public AlgaeEffectorCommand(AlgaeEffector algaeEffector, Joystick stick) {

    this.algaeEffector = algaeEffector;
    this.stick = stick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (stick.getRawButton(Constants.AlgaeConstants.ALGAE_INTAKE)) {
      algaeEffector.intake();
    }

    else if (stick.getRawButton(Constants.AlgaeConstants.ALGAE_OUTTAKE)) {
      algaeEffector.expel();
    } 

    else if (stick.getPOV() == 270) {
      algaeEffector.slowExpel();
    }
    
    else {
      algaeEffector.stopIntake();
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

  public class IntakeAlgae extends Command {
    public IntakeAlgae() {
      addRequirements();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      algaeEffector.intake();
    }
  }

  public class OuttakeAlgae extends Command {
    public OuttakeAlgae() {
      addRequirements();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      algaeEffector.expel();
    }
  }

  public class StopIntakeAlgae extends Command {
    public StopIntakeAlgae() {
      addRequirements();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      algaeEffector.stopIntake();
    }
  }
}
