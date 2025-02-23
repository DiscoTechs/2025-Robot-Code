// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralEffector extends SubsystemBase {

  private final SparkMax leftIntakeMotor;
  private final SparkMax rightIntakeMotor;

  /** Creates a new AlgaeEffector. */
  public CoralEffector() {
    leftIntakeMotor = new SparkMax(Constants.DriveConstants.kLeftCoralEffectorMotorPort, MotorType.kBrushless);
    rightIntakeMotor = new SparkMax(Constants.DriveConstants.kRightCoralEffectorMotorPort, MotorType.kBrushless);
  }

  public void intake() {
    leftIntakeMotor.set(0.5);
    rightIntakeMotor.set(0.5);
  }

  public void expel() {
    leftIntakeMotor.set(-0.5);
    rightIntakeMotor.set(-0.5);
  }

  public void stop() {
    leftIntakeMotor.set(0.0);
    rightIntakeMotor.set(0.0);
  }

  //rightIntakeMotor.follow(leftIntakeMotor);


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
