// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OutAndBack extends SequentialCommandGroup {
  
  /** Creates a new OutAndBack. */
  public OutAndBack(SwerveSubsystem swerve, double t) {

    addCommands(new SimpleAuto(swerve, .5, 0, 0).withTimeout(t));
    addCommands(new AutoTurn(swerve, 90));
    addCommands(new SimpleAuto(swerve, 0, -0.5, 0).withTimeout(t));
    addCommands(new AutoTurn(swerve, -90));
  }
}
