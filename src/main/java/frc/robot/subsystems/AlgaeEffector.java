// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeEffector extends SubsystemBase {

  private final SparkMax leftIntakeMotor;
  private final SparkMax rightIntakeMotor;

  /** Creates a new AlgaeEffector. */
  public AlgaeEffector() {
    leftIntakeMotor = new SparkMax(Constants.DriveConstants.kLeftAlgaeEffectorMotorPort, MotorType.kBrushless);
    rightIntakeMotor = new SparkMax(Constants.DriveConstants.kRightAlgaeEffectorMotorPort, MotorType.kBrushless);
  }

  public void intake() {
    leftIntakeMotor.set(0.8);
    rightIntakeMotor.set(0.8);
  }

  public void expel() {
    leftIntakeMotor.set(-0.8);
    rightIntakeMotor.set(-0.8);
  }

  public void stopIntake() {
    leftIntakeMotor.set(0.0);
    rightIntakeMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
