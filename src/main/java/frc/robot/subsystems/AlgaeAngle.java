// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DutyCycleEncoder; //on-shaft encoder/through bore encoder
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeAngle extends SubsystemBase {

  private final SparkMax angleMotor;
  private final DutyCycleEncoder encoder;
  private final SparkClosedLoopController pidController;

  /** Creates a new AlgaeEffector. */
  public AlgaeAngle() {
    angleMotor = new SparkMax(Constants.AlgaeConstants.kAlgaeAngleMotorPort, MotorType.kBrushless);
    encoder = new DutyCycleEncoder(1, 2*Math.PI, 0.0); // change according to this specification: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/DutyCycleEncoder.html
    
    // first pass at PID
    pidController = angleMotor.getClosedLoopController();

    //^ Adjust this according to testing
  }

  public void angleUp() {
    if (getEncoder() <= (Constants.AlgaeConstants.kMaxEncoderValue - Constants.AlgaeConstants.kAlgaeDelta)) {
      angleMotor.set(Constants.AlgaeConstants.kAlgaeAngleSpeed);
    }
    else {
      stopAngle();
    }
  }

  public void angleDown() {
    if (getEncoder() >= (Constants.AlgaeConstants.kMinEncoderValue + Constants.AlgaeConstants.kAlgaeDelta)) {
      angleMotor.set(-Constants.AlgaeConstants.kAlgaeAngleSpeed);
    }
    else {
      stopAngle(); 
    }
  }

  public void stopAngle()  {
    angleMotor.set(0.0);
  }

  public double getEncoder() {
    return encoder.get();
  }

  // public void angleUpToDefault() {
  //   while(encoder.get() <= (Constants.AlgaeConstants.kDefaultEncoderValue - Constants.AlgaeConstants.kAlgaeDelta)) { //just means while not within the range (default - delta, default + delta) [centered around default], then angle up the plate
  //     angleMotor.set(-Constants.AlgaeConstants.kAlgaeAngleSpeed); //adjust SIGN (espcially) and speed (now in constants for ease) as needed
  //   }
  // }

  // public void angleDownToDefault() {
  //   while(encoder.get() >= (Constants.AlgaeConstants.kDefaultEncoderValue - Constants.AlgaeConstants.kAlgaeDelta)) { //just means while not within the range (default - delta, default + delta) [centered around default], then angle down the plate
  //     angleMotor.set(Constants.AlgaeConstants.kAlgaeAngleSpeed); //adjust SIGN (espcially) and speed (now in constants for ease) as needed
  //   } 
  // }

  // public void angleMinSet() {
  //   while(getEncoder() >= Constants.AlgaeConstants.kMinEncoderValue) {
  //     angleDown();
  //   }
  // }

  // public void angleMaxSet() {
  //   while(getEncoder() <= Constants.AlgaeConstants.kMaxEncoderValue) {
  //     angleUp();
  //   }
  // }

  public void goToScoringAngle() {
    if (getEncoder() <= Constants.AlgaeConstants.kScoringEncoderValue) {
      angleUp();
    }
    else if (getEncoder() >= Constants.AlgaeConstants.kScoringEncoderValue) {
      angleDown();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  double pos = 2.4;

  public void positionDrive() {
    // Working on this - Mr. Z
    pidController.setReference(pos, SparkBase.ControlType.kPosition);
  }

}
