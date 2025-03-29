// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterElevator;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReefAuto extends SequentialCommandGroup {
  /** Creates a new CustomAuto. */

  public final int LEFT = 1;
  public final int CENTER = 2;
  public final int RIGHT = 3;
  

  public ReefAuto(SwerveSubsystem swerve, ShooterElevator shooterElevator, int position) {

    // Move Forward (all postions)
    double speed = 0.7;

    addCommands(new SimpleAuto(swerve, speed, 0, 0).withTimeout(1));

    // turn - position based
    if (position == 1) {
      speed = 0.8;
    } else if (position == 3){
      speed = -0.8;
    } else {
      speed = 0.0;
    }

    addCommands(new SimpleAuto(swerve, 0, 0.0, speed).withTimeout(1.8));
    
    // drive to reef (position based)
    if (position == 1) {
      speed = 0.8;
    } else if (position == 3){
      speed = 0.8;
    } else {
      speed = 0.3;
    }
    addCommands(new SimpleAuto(swerve, speed, 0.0, 0.0).withTimeout(8));
    
    // addCommands(new LimeLightAuto(swerve, 0).withTimeout(4));

    // Drop Coral
    addCommands(new CoralDrop(shooterElevator).withTimeout(1));
    
  }
}
