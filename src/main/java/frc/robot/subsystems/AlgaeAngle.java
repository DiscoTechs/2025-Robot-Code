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
    while(!(getEncoder() >= Constants.AlgaeConstants.kMaxEncoderValue - Constants.AlgaeConstants.kAlgaeDelta)) {
      angleMotor.set(-Constants.AlgaeConstants.kAlgaeAngleSpeed); //temporary --> have to change to closedLoop/on-axel encoder
    }
  }

  public void angleDown() {
    while(!(getEncoder() <= Constants.AlgaeConstants.kMinEncoderValue + Constants.AlgaeConstants.kAlgaeDelta)) {
      angleMotor.set(Constants.AlgaeConstants.kAlgaeAngleSpeed); //temporary --> have to change to closedLoop/on-axel encoder
    }
  }

  public void stopAngle()  {
    angleMotor.set(0.0);
  }

  public double getEncoder() {
    return encoder.get();
  }

  public boolean inRange() {
    return ((encoder.get() >= Constants.AlgaeConstants.kDefaultEncoderValue - Constants.AlgaeConstants.kAlgaeDelta) && (encoder.get() <= Constants.AlgaeConstants.kDefaultEncoderValue + Constants.AlgaeConstants.kAlgaeDelta));
  }

  public void angleUpToDefault() {
    while(!inRange()) { //just means while not within the range (default - delta, default + delta) [centered around default], then angle up the plate
      angleMotor.set(-Constants.AlgaeConstants.kAlgaeAngleSpeed); //adjust SIGN (espcially) and speed (now in constants for ease) as needed
    }
  }

  public void angleDownToDefault() {
    while(!inRange()) { //just means while not within the range (default - delta, default + delta) [centered around default], then angle down the plate
      angleMotor.set(Constants.AlgaeConstants.kAlgaeAngleSpeed); //adjust SIGN (espcially) and speed (now in constants for ease) as needed
    }
  }

  public void angleMinSet() {
    while(getEncoder() >= Constants.AlgaeConstants.kMinEncoderValue) {
      angleDown();
    }
  }

  public void angleMaxSet() {
    while(getEncoder() <= Constants.AlgaeConstants.kMaxEncoderValue) {
      angleUp();
    }
  }

  public void setDefaultAngle() {
    if (getEncoder() <= Constants.AlgaeConstants.kDefaultEncoderValue) {
      angleUpToDefault();
    }
    else if (getEncoder() >= Constants.AlgaeConstants.kDefaultEncoderValue) {
      angleDownToDefault();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
