// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AutoCommands.LimeLightAuto;
import frc.robot.commands.AutoCommands.OutAndBack;
import frc.robot.commands.AutoCommands.SimpleAuto;
import frc.robot.subsystems.AlgaeEffector;
import frc.robot.subsystems.SwerveSubsystem; 

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);

  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final AlgaeEffector algaeEffector = new AlgaeEffector();

  private final SendableChooser<Command> autoChooser;
  public static final SendableChooser<Integer> limelightFilterChooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !driverJoystick.getRawButton(OIConstants.RIGHT_BUMPER),
      () -> driverJoystick.getRawButton(OIConstants.XBX_B),
      () -> driverJoystick.getRawButton(OIConstants.XBX_A),
      driverJoystick
    ));

    algaeEffector.setDefaultCommand(new AlgaeCommand(algaeEffector, driverJoystick));

    autoChooser = AutoBuilder.buildAutoChooser();//new SendableChooser<>(); 
    Command auto1 = new SimpleAuto(swerveSubsystem, 1, 0, 0);
    Command auto2 = new SimpleAuto(swerveSubsystem, 0, 1, 0);
    Command outAndBack = new OutAndBack(swerveSubsystem, 2);
    Command limeLightAuto = new LimeLightAuto(swerveSubsystem, 2);
    //PathPlannerAuto testAuto = new PathPlannerAuto("Test Auto 2.auto");

    autoChooser.setDefaultOption("x--", auto1);
    autoChooser.addOption("-y-", auto2);
    autoChooser.addOption("Out And Back", outAndBack);
    autoChooser.addOption("LimeLight Auto", limeLightAuto);
    //autoChooser.addOption("Test Auto", testAuto);
    SmartDashboard.putData("Auto Choices", autoChooser);

    limelightFilterChooser.setDefaultOption("None", 0);
    for (int i = 1; i <= 15; i++) {
      limelightFilterChooser.addOption("ID: " + i, i);
    }

    SmartDashboard.putData("April Tags Filter", limelightFilterChooser);
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new Joystick(m_exampleSubsystem));

    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  // /**
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  *
  //  * @return the command to run in autonomous
  //  */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    /*try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerAuto path = PathPlannerAuto.fromAutoFile("Arjun's Test Path.path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }*/

    try {
        return new PathPlannerAuto("Test Auto 2");
    } catch (Exception e) {
        DriverStation.reportError("Error loading autonomous path: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }    //return autoChooser.getSelected();
  }

  public int getAprilTagFilter() {
    // Returns the current April Tag Filter. Returns 0 if none.
    return limelightFilterChooser.getSelected();
  }
}
