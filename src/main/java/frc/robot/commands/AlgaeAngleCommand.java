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
    //System.out.println("AA: " + algaeAngle.getEncoder());

    if (stick.getRawButton(8)) {
      if (stick.getRawButton(Constants.AlgaeConstants.ALGAE_ANGLE_STAY_MIN)) {
          algaeAngle.moveArm(-0.15);
      }
      else if (stick.getRawButton(Constants.AlgaeConstants.ALGAE_ANGLE_STAY_MAX)) {
        algaeAngle.moveArm(0.15);
      }
      else if (stick.getRawButton(7)) {
        algaeAngle.setAbsoluteLowPosition();
      }
    }

    else {
      if (stick.getRawButton(Constants.AlgaeConstants.ALGAE_ANGLE_STAY_MAX)) {
        algaeAngle.angleUp();
        //algaeAngle.moveArm(0.2);
      }

      else if (stick.getRawButton(Constants.AlgaeConstants.ALGAE_ANGLE_STAY_MIN)) {
          algaeAngle.angleDown();
          //algaeAngle.moveArm(-0.1);
      }

      else if (stick.getRawButton(Constants.AlgaeConstants.ALGAE_ANGLE_SCORING)) {
        algaeAngle.goToScoringAngle();
      }

      else if (stick.getRawButton(2)) {
        algaeAngle.goToFloorIntake();
      }

      else if (stick.getRawButton(7)) {
        algaeAngle.setLowPosition();
      }

      else {
        algaeAngle.stopAngle();
        // algaeAngle.moveArm(0.02);
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

  public class AlgaeDefaultSet extends Command {
    public AlgaeDefaultSet() {
      addRequirements();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      algaeAngle.goToScoringAngle();
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
