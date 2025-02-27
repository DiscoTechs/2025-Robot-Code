// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// public class Climber extends SubsystemBase{
//     private final SparkMax climbMotor;
    
//     public Climber() {
//         climbMotor = new SparkMax(Constants.ClimberConstants.kClimberMotorPort, MotorType.kBrushless);
//     }

//     public void climberUp() {
//         climbMotor.set(0.5); //temporary --> have to change to closedLoop
//     }

//     public void climberDown() {
//         climbMotor.set(-0.5); //temporary --> have to change to closedLoop
//     }

//     public void climberStop() {
//         climbMotor.set(0);
//     }
// }
