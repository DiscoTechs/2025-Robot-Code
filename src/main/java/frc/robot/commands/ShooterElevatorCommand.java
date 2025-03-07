// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterElevator;

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
    //intake sequence
    boolean exitOuterIf = false;
    if (stick.getRawButton(Constants.CoralConstants.INITIATE_HUMAN_PLAYER_SEQUENCE)) {
      shooterElevator.goToFirstLevel();

      while (!shooterElevator.isSensorDetected(shooterElevator.coralSensor)) {
        if (stick.getRawButton(Constants.CoralConstants.ESCAPE)) {
          exitOuterIf = true;
          break;
        }
        shooterElevator.outtakeCoral();
      }

      if (!exitOuterIf) {
        while (shooterElevator.isSensorDetected(shooterElevator.coralSensor)) {
          shooterElevator.outtakeCoral();
          shooterElevator.stopElevator();
        }

        shooterElevator.stopShooter();
      }

      shooterElevator.stopShooter();
    }
    //intake and outtake coral
    if (stick.getRawButton(Constants.CoralConstants.CORAL_INTAKE)) {
      shooterElevator.intakeCoral();
    }
    else if (stick.getRawButton(Constants.CoralConstants.CORAL_OUTTAKE)) {
      shooterElevator.outtakeCoral();
    }
    else {
      shooterElevator.stopShooter();
    }
    //stop elevator if coral blocked or reached top limit
    if (shooterElevator.isTopLimitReached() || shooterElevator.isSensorDetected(shooterElevator.coralSensor)) {
      shooterElevator.stopElevator();
    }
    //level 1
    else if (stick.getRawButton(Constants.ElavatorConstants.LEVEL_1)) {
      shooterElevator.goToFirstLevel();
      if (!stick.getRawButton(Constants.AlgaeConstants.ALGAE_INTAKE)) {
        shooterElevator.outtakeCoral();
      }
    }
    //level 2
    else if (stick.getRawButton(Constants.ElavatorConstants.LEVEL_2)) {
      shooterElevator.goToSecondLevel();
      if (!stick.getRawButton(Constants.AlgaeConstants.ALGAE_INTAKE)) {
        shooterElevator.outtakeCoral();
      }
    }
    //level 3
    else if (stick.getRawButton(Constants.ElavatorConstants.LEVEL_3)) {
      shooterElevator.goToThirdLevel();
      if (!stick.getRawButton(Constants.AlgaeConstants.ALGAE_INTAKE)) {
        shooterElevator.outtakeCoral();
      }
    }
    //level 4
    else if (stick.getRawButton(Constants.ElavatorConstants.LEVEL_4)) {
      shooterElevator.goToFourthLevel();
      if (!stick.getRawButton(Constants.AlgaeConstants.ALGAE_INTAKE)) {
        shooterElevator.outtakeCoral();
      }
    }
    //manual control
    else if (stick.getRawButton(Constants.ElavatorConstants.USE_MANUAL_CONTROL_JOYSTICK)) {
      double speed = -stick.getRawAxis(Constants.ElavatorConstants.MANUAL_CONTROL_AXIS);

      if (shooterElevator.isTopLimitReached() || shooterElevator.isSensorDetected(shooterElevator.coralSensor)) {
        shooterElevator.stopElevator();
      }

      else if (speed < 0) {
        if (shooterElevator.isSensorDetected(shooterElevator.Level1Sensor) || shooterElevator.isSensorDetected(shooterElevator.coralSensor)) {
          shooterElevator.stopElevator();
        }
        else {
          shooterElevator.moveElevatorDown();
        }
      } 

      else if (speed > 0) {
        if (shooterElevator.isTopLimitReached() || shooterElevator.isSensorDetected(shooterElevator.coralSensor)) {
          shooterElevator.stopElevator();
        }
        else {
          shooterElevator.moveElevatorUp();
        }
      }

      else {
        shooterElevator.stopElevator();
      }
    }
    //default to first level
    else {
      shooterElevator.goToFirstLevel();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public class ElevatorLevelOne extends Command {
    public ElevatorLevelOne() {
      addRequirements();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      shooterElevator.goToFirstLevel();
    }
  }

  public class ElevatorLevelTwo extends Command {
    public ElevatorLevelTwo() {
      addRequirements();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      shooterElevator.goToSecondLevel();
    }
  }

  public class ElevatorLevelThird extends Command {
    public ElevatorLevelThird() {
      addRequirements();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      shooterElevator.goToThirdLevel();
    }
  }

  public class ElevatorLevelForth extends Command {
    public ElevatorLevelForth() {
      addRequirements();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      shooterElevator.goToFourthLevel();
    }
  }

  public class IntakeCoral extends Command {
    public IntakeCoral() {
      addRequirements();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      shooterElevator.intakeCoral();
    }
  }

  public class OuttakeCoral extends Command {
    public OuttakeCoral() {
      addRequirements();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      shooterElevator.outtakeCoral();
    }
  }

  public class SequenceIntake extends Command {
    public SequenceIntake() {
      addRequirements();
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      shooterElevator.intakeSequence();
    }
  }
}