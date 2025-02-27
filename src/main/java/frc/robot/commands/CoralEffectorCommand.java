// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CoralEffector;
// import frc.robot.Constants;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class CoralEffectorCommand extends Command {

//   private final CoralEffector coralEffector;
//   private final Joystick stick;

//   /** Creates a new CoralCommand. */
//   public CoralEffectorCommand(CoralEffector coralEffector, Joystick stick) {

//     this.coralEffector = coralEffector;
//     this.stick = stick;

//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(coralEffector);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (stick.getRawButton(Constants.CoralConstants.CORAL_INTAKE)) {
//       coralEffector.intake();
//     } else if (stick.getRawButton(Constants.CoralConstants.CORAL_OUTTAKE)) {
//       coralEffector.expel();
//     } else {
//       coralEffector.stop();
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
