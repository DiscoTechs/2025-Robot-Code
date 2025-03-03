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
    if (stick.getRawButton(Constants.CoralConstants.keepGoing)) {
      shooterElevator.firstLevel();
      while (!shooterElevator.detectCoral() && !stick.getRawButton(Constants.CoralConstants.ESCAPE)) {
        shooterElevator.outtake();
      }
      while (shooterElevator.detectCoral() && !stick.getRawButton(Constants.CoralConstants.ESCAPE)) {
        shooterElevator.outtake();
        shooterElevator.stopElevator();
      }
      shooterElevator.stopShooter();
    }

    //elevator movement
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

    // coral shooter
    if (stick.getRawButton(Constants.CoralConstants.CORAL_INTAKE)) {
      shooterElevator.intake();
    } else if (stick.getRawButton(Constants.CoralConstants.CORAL_OUTTAKE)) {
      shooterElevator.outtake();
    } else {
      shooterElevator.stopShooter();
    }

    // manual control of elevator
    double speed = -stick.getRawAxis(Constants.ElavatorConstants.MANUAL_CONTROL_AXIS); // potnetially change sign from
                                                                                       // psotive to negative
    if (shooterElevator.detectCoral() || shooterElevator.reachedLimit()) {
      shooterElevator.stopElevator();
    } else if (speed < 0 && (!shooterElevator.L1getSensorValue())) {
      shooterElevator.moveDown();
    } else if (speed > 0) {
      shooterElevator.moveUp();
    } else {
      shooterElevator.stopElevator();
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
      shooterElevator.firstLevel();
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
      shooterElevator.secondLevel();
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
      shooterElevator.thirdLevel();
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
      shooterElevator.fourthLevel();
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
      shooterElevator.intake();
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
      shooterElevator.outtake();
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