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
   //^ Adjust this according to testing
  }

  public void angleUp() {
    while (getEncoder() <= Constants.AlgaeConstants.kMaxEncoderValue) {
      angleMotor.set(Constants.AlgaeConstants.kAlgaeAngleSpeed); //temporary --> have to change to closedLoop/on-axel encoder AND maybe sign
    }
  }

  public void angleDown() {
    while (getEncoder() >= Constants.AlgaeConstants.kMinEncoderValue) {
      angleMotor.set(-Constants.AlgaeConstants.kAlgaeAngleSpeed); //temporary --> have to change to closedLoop/on-axel encoder AND maybe sign
    }
  }

  public void stopAngle()  {
    angleMotor.set(0.0);
  }

  public double getEncoder() {
    return encoder.get();
  }

  public void setDefaultAngle() {
    if (getEncoder() <= Constants.AlgaeConstants.kDefaultEncoderValue) {
      while (!(getEncoder() <= Constants.AlgaeConstants.kDefaultEncoderValue)) { //maybe chnage greater than/less than sign according to how the encoder is reading values
        angleUp();
      }
    }
    else if (getEncoder() >= Constants.AlgaeConstants.kDefaultEncoderValue) {
      while (!(getEncoder() >= Constants.AlgaeConstants.kDefaultEncoderValue)) { //maybe chnage greater than/less than sign according to how the encoder is reading values
        angleDown();
      }
    }
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
