// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DegreeMotor extends Command {
  private final Joystick stick;
  private final SparkMax motor;
  private final SparkClosedLoopController PIDController;

  /** Creates a new DegreeMotor. */
  public DegreeMotor(Joystick stick) {
    this.motor = new SparkMax(Constants.DriveConstants.kAlgaeEffectorMotorPort, MotorType.kBrushless);
    this.stick = stick;

    this.PIDController = this.motor.getClosedLoopController();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.PIDController

    if (stick.getRawButton(1)) {
      motor.(0.8);
    } else if (stick.getRawButton(2)) {
      motor.set(-0.8);
    } else {
      motor.set(0.0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = getPosition();
    PIDController.setReference
  }

  public double getPosition() {
    return motor.getEncoder().getPosition();
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
