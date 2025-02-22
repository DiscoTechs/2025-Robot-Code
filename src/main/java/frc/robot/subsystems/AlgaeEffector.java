// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeEffector extends SubsystemBase {

  private final SparkMax intakeMotor;
  private final SparkMax angleMotor;

  /** Creates a new AlgaeEffector. */
  public AlgaeEffector() {
    intakeMotor = new SparkMax(Constants.DriveConstants.kAlgaeEffectorMotorPort, MotorType.kBrushless);
    angleMotor = new SparkMax(Constants.DriveConstants.kAlgaeAngleMotorPort, MotorType.kBrushless);
  }

  public void intake() {
    intakeMotor.set(0.8);
  }

  public void expel() {
    intakeMotor.set(-0.8);
  }

  public void stopIntake() {
    intakeMotor.set(0.0);
  }
//Possibly make a new SubSystem for algaeAngle 
  public void angleUp() {
    angleMotor.set(0.3); //temporary --> have to change to closedLoop
  }

  public void angleDown() {
    angleMotor.set(-0.3); //temporary --> have to change to closedLoop
  }

  public void stopAngle()  {
    angleMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
