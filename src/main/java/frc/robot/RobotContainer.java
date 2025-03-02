// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.BiConsumer;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlgaeAngleCommand;
import frc.robot.commands.AlgaeEffectorCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.ShooterElevatorCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AutoCommands.LimeLightAuto;
import frc.robot.commands.AutoCommands.OutAndBack;
import frc.robot.commands.AutoCommands.SimpleAuto;
import frc.robot.subsystems.AlgaeAngle;
import frc.robot.subsystems.AlgaeEffector;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ShooterElevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.CoralPlateAngleCommand;
import frc.robot.subsystems.CoralPlateAngle;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);
  private final Joystick buttonBox = new Joystick(OIConstants.kButtonBoxPort);

  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  //ALGAE
  private final AlgaeEffector algaeEffector = new AlgaeEffector();
  //private final AlgaeAngle algaeAngle = new AlgaeAngle();

  // //CORAL
  //private final CoralPlateAngle coralPlateAngle = new CoralPlateAngle();

  // //CLIMBER
  //private final Climber climber = new Climber();

  //SHOOTER-ELEVATOR
  //private final ShooterElevator shooterElevator = new ShooterElevator();

  private final SendableChooser<Command> autoChooser;
  public static final SendableChooser<Integer> limelightFilterChooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureRobot();


    //printAbsoluteEncoder();

    //FOR PUTTING COMMANDS AS DROP-DOWN OPTIONS IN UI:

    //swerve = new Swerve();
    //exampleSubsystem = new ExampleSubsystem();

    // Register Named Commands
    //NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
    //NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
    //NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());

    // Do all other initialization

    //configureButtonBindings();

    // ...
    
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

    algaeEffector.setDefaultCommand(new AlgaeEffectorCommand(algaeEffector, operatorJoystick));
    //algaeAngle.setDefaultCommand(new AlgaeAngleCommand(algaeAngle, operatorJoystick));

    //coralPlateAngle.setDefaultCommand(new CoralPlateAngleCommand(coralPlateAngle, operatorJoystick)); //here, for example, I think I can change to button box after it's configed

    //climber.setDefaultCommand(new ClimberCommand(climber, operatorJoystick));

    //shooterElevator.setDefaultCommand(new ShooterElevatorCommand(shooterElevator, operatorJoystick));
  
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

  /*public void printAbsoluteEncoder() {
    System.out.println("Absolute Encoder Left Front: " + frontLeft.getAbsoluteEncoderDeg());
    System.out.println("Absolute Encoder Right Front: " + frontRight.getAbsoluteEncoderDeg());
    System.out.println("Absolute Encoder Left Back" + backLeft.getAbsoluteEncoderDeg());
    System.out.println("Absolute Encoder Right Back" + backRight.getAbsoluteEncoderDeg());
  }*/

  public void configureRobot() {
    RobotConfig config = null;
    try {
        config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
        e.printStackTrace();
    }

    AutoBuilder.configure(
        swerveSubsystem::getPose2d,
        swerveSubsystem::resetPose,
        swerveSubsystem::getSpeeds,
        (speeds, feedforwards) -> swerveSubsystem.driveRobotRelative(speeds),
        Constants.AutoConstants.holonomicDriveController,
        config,
        () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }

            return false;
        },
        swerveSubsystem
    );
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
        return new PathPlannerAuto("Blue 2 To Reef Auto");
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
