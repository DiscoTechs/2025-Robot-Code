// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.Elevator;
// import frc.robot.Constants;
// import edu.wpi.first.wpilibj.Joystick;


// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class ElevatorCommand extends Command {

//   private final Elevator elevator;
//   private final Joystick stick;

//   /** Creates a new AlgaeCommand. */
//   public ElevatorCommand(Elevator elevator, Joystick stick) {

//     this.elevator = elevator;
//     this.stick = stick;

//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(elevator);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (elevator.getSensorValue()) {
//       elevator.stop();
//     } else if (stick.getRawButton(Constants.ElavatorConstants.L1)) {
//       elevator.firstLevel();
//     } else if (stick.getRawButton(Constants.ElavatorConstants.L2)) {
//       elevator.secondLevel();
//     } else if (stick.getRawButton(Constants.ElavatorConstants.L3)) {
//       elevator.thirdLevel();
//     } else if (stick.getRawButton(Constants.ElavatorConstants.L4)) {
//       elevator.fourthLevel();
//     } else {
//       elevator.stop();
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
