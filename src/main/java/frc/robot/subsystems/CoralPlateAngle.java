// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder; //on-shaft encoder/through bore encoder

public class CoralPlateAngle extends SubsystemBase {

  private final SparkMax angleMotor;
  //private final DutyCycleEncoder encoder;

  /** Creates a new CoralAngle. */
  public CoralPlateAngle() {
    angleMotor = new SparkMax(Constants.CoralConstants.kCoralPlateAngleMotorPort, MotorType.kBrushless);
    //encoder = new DutyCycleEncoder(4, 2*Math.PI, 0.0); // change according to this specification: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/DutyCycleEncoder.html
    // also, make a constant for default encoder value so I don't have to keep being changed
    //^ Adjust this according to testing
  }
 
  public void angleUp() {
    angleMotor.set(Constants.CoralConstants.kAngleSpeed);
  }

  public void angleDown() {
    angleMotor.set(-Constants.CoralConstants.kAngleSpeed);
  }

  public void stopAngle() {
    angleMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
