// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeAngle;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeAngleCommand extends Command {

  private final AlgaeAngle algaeAngle;
  private final Joystick stick;

  /** Creates a new AlgaeCommand. */
  public AlgaeAngleCommand(AlgaeAngle algaeAngle, Joystick stick) {

    this.algaeAngle = algaeAngle;
    this.stick = stick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (algaeAngle.getEncoder() >= Constants.AlgaeConstants.kMaxEncoderValue || algaeAngle.getEncoder() <= Constants.AlgaeConstants.kMinEncoderValue) {
      algaeAngle.stopAngle();
    } else if(stick.getRawButton(Constants.AlgaeConstants.ALGAE_ANGLE_UP)) {
      algaeAngle.angleUp();
    } else if (stick.getRawButton(Constants.AlgaeConstants.ALGAE_ANGLE_DOWN)) {
      algaeAngle.angleDown();
    } else {
      algaeAngle.stopAngle();
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

  public class AlgaeUpAngle extends Command {
    public AlgaeUpAngle() {
      addRequirements();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      algaeAngle.angleUp();
    }
  }

  public class AlgaeDownAngle extends Command {
    public AlgaeDownAngle() {
      addRequirements();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      algaeAngle.angleDown();
    }
  }

  public class AlgaeHighSet extends Command {
    public AlgaeHighSet() {
      addRequirements();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      algaeAngle.setHighAngle();
    }
  }

  public class AlgaeLowSet extends Command {
    public AlgaeLowSet() {
      addRequirements();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      algaeAngle.setLowAngle();
    }
  }

  public class AlgaeAngleStop extends Command {
    public AlgaeAngleStop() {
      addRequirements();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      algaeAngle.stopAngle();
    }
  }




}
