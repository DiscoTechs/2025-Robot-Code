// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterElevatorCommand extends Command {

  private final ShooterElevator shooterElevator;
  private final Joystick stick;
  private int level = 0;

  private final int[] targets = {0, 30, 40, 50};

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
    SmartDashboard.putNumber("TARGET", 30);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //System.out.println("Elevator Executing" + shooterElevator.getEncoder());

    //intake sequence
    // boolean exitOuterIf = false;
    // if (stick.getRawButton(Constants.CoralConstants.INITIATE_HUMAN_PLAYER_SEQUENCE)) {
    //   shooterElevator.goToFirstLevel();
    //   if (!shooterElevator.isSensorDetected(shooterElevator.coralSensor)) {
    //     if (stick.getRawButton(Constants.CoralConstants.ESCAPE)) {
    //       exitOuterIf = true;
    //     }
    //     shooterElevator.outtakeCoral();
    //   }

    //   if (!exitOuterIf) {
    //     if (shooterElevator.isSensorDetected(shooterElevator.coralSensor)) {
    //       shooterElevator.outtakeCoral();
    //       shooterElevator.stopElevator();
    //     }

    //     shooterElevator.stopShooter();
    //   }

    //   shooterElevator.stopShooter();
    // }

    //intake and outtake coral
    if (stick.getRawAxis(2) > 0.3) {
      shooterElevator.shoot(stick.getRawAxis(2)/2);
    }
    else if (stick.getRawAxis(3) > 0.3 && shooterElevator.isSensorDetected(shooterElevator.coralSensor)) {
      shooterElevator.shoot(stick.getRawAxis(3)/4);
    }
    else {
      shooterElevator.stopShooter();
    }

    if (stick.getPOV() >= 0) {
      level = stick.getPOV() / 90;
    }

    SmartDashboard.putNumber("LEVEL", level);

    // if (stick.getRawButton(8)) {  // 8 is start button 
    //   // target control
    //   double target = targets[level];

    //   double diff = (target - shooterElevator.getEncoder()) * 0.05;  

    //   System.out.println(target + " - " + shooterElevator.getEncoder() + " -> " + diff);

    //   // quick speed limiter
    //   if (diff < -0.3) {
    //     diff = -0.3;
    //   } else if (diff > 0.3) {
    //     diff = 0.3;
    //   }


    //   shooterElevator.moveElevator(diff);

    // } else {
      // Joystick control

      //stop elevator if coral blocked or reached top limit
      double spd = -stick.getRawAxis(1);

      //System.out.println(stick.getPOV());
          
      //System.out.println(shooterElevator.getEncoder());
      if (spd > .1 && shooterElevator.getEncoder() > 56.5) {
        shooterElevator.moveElevator(0.04);
      }
      else if (spd > .1 && shooterElevator.getEncoder() < 47) {
        shooterElevator.moveElevator(spd * 0.5); // was divide by 2, changed to percent multiplier
      } else if (spd > .1 && shooterElevator.getEncoder() > 47) {
        shooterElevator.moveElevator(spd * 0.166);
      }
      else if (spd < -.1 && shooterElevator.getEncoder() < 10) { //10 is the value that we might have to adjust, when slow speed starts
        shooterElevator.moveElevator(spd * 0.125);
      } else if (spd < -.1 && shooterElevator.getEncoder() > 1.5) {
        shooterElevator.moveElevator(spd * 0.25);
      } 
      else {
        shooterElevator.moveElevator(0.04);   // holding voltage
      }

      if (stick.getRawButton(8)) {
        shooterElevator.setLowPosition();
      }

      if (stick.getPOV() == 180) {
        if (shooterElevator.getEncoder() < (Constants.AlgaeConstants.EncoderBetweenL2AndL3 - Constants.AlgaeConstants.Delta)) {
          shooterElevator.moveElevator(0.2);
        }
        else if (shooterElevator.getEncoder() > (Constants.AlgaeConstants.EncoderBetweenL2AndL3 + Constants.AlgaeConstants.Delta)) {
         shooterElevator.moveElevator(-0.2);
        }
        else {
           shooterElevator.moveElevator(0.04);
        }
     }
     else if (stick.getPOV() == 0) {
      if (shooterElevator.getEncoder() < (Constants.AlgaeConstants.EncoderBetweenL3AndL4 - Constants.AlgaeConstants.Delta)) {
        shooterElevator.moveElevator(0.2);
      }
      else if (shooterElevator.getEncoder() > (Constants.AlgaeConstants.EncoderBetweenL3AndL4 + Constants.AlgaeConstants.Delta)) {
        shooterElevator.moveElevator(-0.2);
      }
      else {
        shooterElevator.moveElevator(0.04);
      }
    }
  }
}

    // if (shooterElevator.isTopLimitReached() || shooterElevator.isSensorDetected(shooterElevator.coralSensor)) {
    //   System.out.println("Sensor Limit STOPPED" + System.currentTimeMillis());
    //   shooterElevator.stopElevator();
    // }

    // else if (stick.getRawButton(7)) {

    //   double speed = -stick.getRawAxis(Constants.ElavatorConstants.MANUAL_CONTROL_AXIS);
    //   System.out.println("Manual Control " + shooterElevator.getEncoder());
      
    //   if (shooterElevator.isTopLimitReached() || shooterElevator.isSensorDetected(shooterElevator.coralSensor)) {
    //     shooterElevator.stopElevator();

    //     System.out.println("STOPPING: CORAL OR TOP");
    //   }

    //   else if (speed < 0) {
    //       if (shooterElevator.isSensorDetected(shooterElevator.Level1Sensor) || shooterElevator.isSensorDetected(shooterElevator.coralSensor)) {
    //         shooterElevator.stopElevator();
    //         System.out.println("STOPPING ELEVATOR: BOTTOM");
    //       } 
    //       else {
    //         shooterElevator.moveElevatorDown();
    //         System.out.println("GOING DOWN");
    //       }
    //   }

    //   else if (speed > 0) {
    //       if (shooterElevator.isTopLimitReached() || shooterElevator.isSensorDetected(shooterElevator.coralSensor)) {
    //         shooterElevator.stopElevator();
    //         System.out.println("STOPPING ELEVATOR: TOP OR CORAL");
    //       } 
    //       else {
    //         shooterElevator.moveElevatorUp();
    //         System.out.println("GOING UP");
    //       }
    //   }

    //   else {
    //     shooterElevator.stopElevator();
    //     System.out.println("STOP");
    //   }
   // }

    // //level 1
    // else if (stick.getRawButton(Constants.ElavatorConstants.LEVEL_1)) {
    //   shooterElevator.goToFirstLevel();
    //   if (!stick.getRawButton(Constants.AlgaeConstants.ALGAE_INTAKE)) {
    //     shooterElevator.outtakeCoral();
    //   }
    // }
    // //level 2
    // else if (stick.getRawButton(Constants.ElavatorConstants.LEVEL_2)) {
    //   shooterElevator.goToSecondLevel();
    //   if (!stick.getRawButton(Constants.AlgaeConstants.ALGAE_INTAKE)) {
    //     shooterElevator.outtakeCoral();
    //   }
    // }
    // //level 3
    // else if (stick.getRawButton(Constants.ElavatorConstants.LEVEL_3)) {
    //   shooterElevator.goToThirdLevel();
    //   if (!stick.getRawButton(Constants.AlgaeConstants.ALGAE_INTAKE)) {
    //     shooterElevator.outtakeCoral();
    //   }
    // }
    // //level 4
    // else if (stick.getRawButton(Constants.ElavatorConstants.LEVEL_4)) {
    //   shooterElevator.goToFourthLevel();
    //   if (!stick.getRawButton(Constants.AlgaeConstants.ALGAE_INTAKE)) {
    //     shooterElevator.outtakeCoral();
    //   }

    // } 

    // //default to first level
    // else {
    //   shooterElevator.goToFirstLevel();
    // }
  
  // }

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {
  // }

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   System.out.println("Elevator finished?");
  //   return false;
  // }

  // // public class ElevatorLevelOne extends Command {
  // //   public ElevatorLevelOne() {
  // //     addRequirements();
  // //   }

  // //   @Override
  // //   public void initialize() {}

  // //   @Override
  // //   public void execute() {
  // //     shooterElevator.goToFirstLevel();
  // //   }
  // // }

  // // public class ElevatorLevelTwo extends Command {
  // //   public ElevatorLevelTwo() {
  // //     addRequirements();
  // //   }

  // //   @Override
  // //   public void initialize() {}

  // //   @Override
  // //   public void execute() {
  // //     shooterElevator.goToSecondLevel();
  // //   }
  // // }

  // // public class ElevatorLevelThird extends Command {
  // //   public ElevatorLevelThird() {
  // //     addRequirements();
  // //   }

  // //   @Override
  // //   public void initialize() {}

  // //   @Override
  // //   public void execute() {
  // //     shooterElevator.goToThirdLevel();
  // //   }
  // // }

  // // public class ElevatorLevelForth extends Command {
  // //   public ElevatorLevelForth() {
  // //     addRequirements();
  // //   }

  // //   @Override
  // //   public void initialize() {}

  // //   @Override
  // //   public void execute() {
  // //     shooterElevator.goToFourthLevel();
  // //   }
  // // }

  // public class IntakeCoral extends Command {
  //   public IntakeCoral() {
  //     addRequirements();
  //   }

  //   @Override
  //   public void initialize() {}

  //   @Override
  //   public void execute() {
  //     shooterElevator.intakeCoral();
  //   }
  // }

  // public class OuttakeCoral extends Command {
  //   public OuttakeCoral() {
  //     addRequirements();
  //   }

  //   @Override
  //   public void initialize() {}

  //   @Override
  //   public void execute() {
  //     shooterElevator.outtakeCoral();
  //   }
  // }

  // public class SequenceIntake extends Command {
  //   public SequenceIntake() {
  //     addRequirements();
  //   }

  //   @Override
  //   public void initialize() {}

  //   @Override
  //   public void execute() {
  //     shooterElevator.intakeSequence();
  //   }
  // }
// }