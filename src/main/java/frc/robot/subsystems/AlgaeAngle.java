// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder; //on-shaft encoder/through bore encoder
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeAngle extends SubsystemBase {

  private final SparkMax angleMotor;
  private final DutyCycleEncoder encoder;

  /** Creates a new AlgaeEffector. */
  public AlgaeAngle() {
    angleMotor = new SparkMax(Constants.AlgaeConstants.kAlgaeAngleMotorPort, MotorType.kBrushless);
    encoder = new DutyCycleEncoder(5, 2*Math.PI, 0.0); // change according to this specification: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/DutyCycleEncoder.html
  }

  public void angleUp() {
    angleMotor.set(0.3); //temporary --> have to change to closedLoop/on-axel encoder
  }

  public void angleDown() {
    angleMotor.set(-0.3); //temporary --> have to change to closedLoop/on-axel encoder
  }

  public void stopAngle()  {
    angleMotor.set(0.0);
  }

  public double getEncoder() {
    return encoder.get();
  }

  public void setHighAngle() {
    while (!(getEncoder() >= Constants.AlgaeConstants.kHighEncoderValue)) {
      angleUp();
    }
  }

  public void setLowAngle() {
    while (!(getEncoder() <= Constants.AlgaeConstants.kLowEncoderValue)) {
      angleDown();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
