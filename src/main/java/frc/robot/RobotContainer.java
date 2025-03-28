// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj.DriverStation;
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
//import frc.robot.commands.CoralPlateAngleCommand;
import frc.robot.commands.ShooterElevatorCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AutoCommands.AutoDrop;
import frc.robot.commands.AutoCommands.CoralDrop;
import frc.robot.commands.AutoCommands.ReefAuto;
import frc.robot.commands.AutoCommands.LimeLightAuto;
import frc.robot.commands.AutoCommands.OutAndBack;
import frc.robot.commands.AutoCommands.SimpleAuto;
import frc.robot.subsystems.AlgaeAngle;
import frc.robot.subsystems.AlgaeEffector;
//import frc.robot.subsystems.Climber;
//import frc.robot.subsystems.CoralPlateAngle;
import frc.robot.subsystems.ShooterElevator;
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
  private final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);
  //private final Joystick buttonBox = new Joystick(OIConstants.kButtonBoxPort);

  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  //ALGAE
  private final AlgaeEffector algaeEffector = new AlgaeEffector();
  private final AlgaeAngle algaeAngle = new AlgaeAngle();

  // //CORAL
  // private final CoralPlateAngle coralPlateAngle = new CoralPlateAngle();

  // //CLIMBER
  // private final Climber climber = new Climber();

  //SHOOTER-ELEVATOR
  private final ShooterElevator shooterElevator = new ShooterElevator();

  private final SendableChooser<Command> autoChooser;
  public static final SendableChooser<Integer> limelightFilterChooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureRobot();

    //printAbsoluteEncoder();

    // FOR PUTTING COMMANDS AS DROP-DOWN OPTIONS IN UI:
    // SAMPLE: KEEP COMMENTED
    // swerve = new Swerve();
    // exampleSubsystem = new ExampleSubsystem();

    // CHANGE TO UNCOMMENTED
    // AlgaeAngle algaeAngle = new AlgaeAngle();
    // AlgaeEffector algaeEffector = new AlgaeEffector();
    
    //Climber climber = new Climber();
    // CoralPlateAngle coralPlateAngle = new CoralPlateAngle();
    // ShooterElevator shooterElevator = new ShooterElevator();
    //SwerveModule swerveModule = new SwerveModule(); //--> likely keep this commented?
    // SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    // SAMPLE: KEEP COMMENTED
    // NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
    // NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
    // NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());
    
    // Register Named Commands: CHANGE TO UNCOMMENTED
    // NamedCommands.registerCommand("algaeHighAngle", algaeAngle.setHighAngle());
    // NamedCommands.registerCommand("algaeLowAngle", algaeAngle.setLowAngle());
    // NamedCommands.registerCommand("algaeIntake", algaeEffector.intake());
    // NamedCommands.registerCommand("algaeOuttake", algaeEffector.expel());

    //FIX CODE BELOW TO REFLECT CODE IN EXTENDED COMMANDS BELOW

    //CoralPlateAngleCommand coralPlateAngleCommand = new CoralPlateAngleCommand(coralPlateAngle, buttonBox);
    //NamedCommands.registerCommand("coralPlateAngleUp", coralPlateAngleCommand.new UpAngle());
    //NamedCommands.registerCommand("coralPlateAngleDown", coralPlateAngleCommand.new DownAngle());
    //NamedCommands.registerCommand("coralPlateDefault", coralPlateAngleCommand.new DefaultAngle());

    // ShooterElevatorCommand shooterElevatorCommand = new ShooterElevatorCommand(shooterElevator, buttonBox);
    // NamedCommands.registerCommand("coralIntake", shooterElevatorCommand.new IntakeCoral());
    // NamedCommands.registerCommand("coralOuttake", shooterElevatorCommand.new OuttakeCoral());
    // NamedCommands.registerCommand("coralIntakeSequence", shooterElevatorCommand.new SequenceIntake());
    // NamedCommands.registerCommand("firstLevel", shooterElevatorCommand.new ElevatorLevelOne());
    // NamedCommands.registerCommand("secondLevel", shooterElevatorCommand.new ElevatorLevelTwo());
    // NamedCommands.registerCommand("thirdLevel", shooterElevatorCommand.new ElevatorLevelThird());
    // NamedCommands.registerCommand("fourthLevel", shooterElevatorCommand.new ElevatorLevelForth());

    // AlgaeAngleCommand algaeAngleCommand = new AlgaeAngleCommand(algaeAngle, operatorJoystick);
    // NamedCommands.registerCommand("algaeDefaultAngle", algaeAngleCommand.new AlgaeDefaultSet());
    // NamedCommands.registerCommand("algaeAngleUp", algaeAngleCommand.new AlgaeUpAngle());
    // NamedCommands.registerCommand("algaeAngleDown", algaeAngleCommand.new AlgaeDownAngle());
    // NamedCommands.registerCommand("algaeAngleStop", algaeAngleCommand.new AlgaeAngleStop());

    // AlgaeEffectorCommand algaeEffectorCommand = new AlgaeEffectorCommand(algaeEffector, operatorJoystick);
    // NamedCommands.registerCommand("algaeIntake", algaeEffectorCommand.new IntakeAlgae());
    // NamedCommands.registerCommand("algaeOuttake", algaeEffectorCommand.new OuttakeAlgae());
    // NamedCommands.registerCommand("algaeIntakeStop", algaeEffectorCommand.new StopIntakeAlgae());

    // Do all other initialization

    //configureButtonBindings();

    // ...
    
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
      () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !driverJoystick.getRawButton(OIConstants.RIGHT_BUMPER),
      () -> driverJoystick.getRawButton(OIConstants.LEFT_BUMPER),
      driverJoystick
    ));

    algaeEffector.setDefaultCommand(new AlgaeEffectorCommand(algaeEffector, operatorJoystick));
    algaeAngle.setDefaultCommand(new AlgaeAngleCommand(algaeAngle, operatorJoystick));
    shooterElevator.setDefaultCommand(new ShooterElevatorCommand(shooterElevator, operatorJoystick));
  
    // AUTONOMOUS
    autoChooser = AutoBuilder.buildAutoChooser();//new SendableChooser<>(); 

    //PathPlannerAuto testAuto = new PathPlannerAuto("Test Auto 2.auto");

    autoChooser.setDefaultOption("Move", new SimpleAuto(swerveSubsystem, 0.4, 0, 0).withTimeout(1.5));

    autoChooser.addOption("Center", new ReefAuto(swerveSubsystem, shooterElevator, 2));
    autoChooser.addOption("Left", new ReefAuto(swerveSubsystem, shooterElevator, 1));
    autoChooser.addOption("Right", new ReefAuto(swerveSubsystem, shooterElevator, 3));

    Command limeLightAuto = new LimeLightAuto(swerveSubsystem, 0);
    autoChooser.addOption("LimeLight Auto", limeLightAuto);

    SmartDashboard.putData("Auto Choices", autoChooser);

    // Limelight Filters
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

    // try {
    //     return new PathPlannerAuto("Slow Auto");
    // } catch (Exception e) {
    //     DriverStation.reportError("Error loading autonomous path: " + e.getMessage(), e.getStackTrace());
    //     return Commands.none();
    // } 
    
    return autoChooser.getSelected();
  }

  public int getAprilTagFilter() {
    // Returns the current April Tag Filter. Returns 0 if none.
    return limelightFilterChooser.getSelected();
  }
}
