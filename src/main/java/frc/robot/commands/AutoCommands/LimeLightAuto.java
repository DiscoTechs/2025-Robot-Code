package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimeLightAuto extends SequentialCommandGroup {
  
  /** Creates a new OutAndBack. */
  public LimeLightAuto(SwerveSubsystem swerve, double t) {
    addCommands(new SimpleAuto(swerve, .5, 0, 0).withTimeout(t));
  }
}