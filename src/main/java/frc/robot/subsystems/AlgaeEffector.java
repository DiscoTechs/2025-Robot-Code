// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeEffector extends SubsystemBase {

  private final SparkMax intakeMotor;

  /** Creates a new AlgaeEffector. */
  public AlgaeEffector() {

    intakeMotor = new SparkMax(31, MotorType.kBrushless);

  }

  public void intake() {
    intakeMotor.set(0.5);
  }

  public void expel() {
    intakeMotor.set(-0.5);
  }

  public void stop() {
    intakeMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
