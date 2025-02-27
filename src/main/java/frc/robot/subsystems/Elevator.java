// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkLowLevel.MotorType;


// public class Elevator extends SubsystemBase {
//     final DigitalInput sensor;
//     private final SparkMax intakeMotor;
    
//     public Elevator() {
//         sensor = new DigitalInput(Constants.ElavatorConstants.ELAVATOR_SENSOR);
//         intakeMotor = new SparkMax(Constants.ElavatorConstants.kElevatorMotorPort, MotorType.kBrushless);
//     }

//     public boolean getSensorValue() {
//         return !sensor.get();
//     }

//     public void firstLevel() {
//         intakeMotor.set(0.1); //temporary --> HAVE to change to closedLoop/on-axel encoder
//     }

//     public void secondLevel() {
//         intakeMotor.set(0.3); //temporary --> HAVE to change to closedLoop/on-axel encoder
//     }

//     public void thirdLevel() {
//         intakeMotor.set(0.7); //temporary --> HAVE to change to closedLoop/on-axel encoder
//     }

//     public void fourthLevel() {
//         intakeMotor.set(0.9); //temporary --> HAVE to change to closedLoop//on-axel encoder
//     }

//     public void stop() {
//         intakeMotor.set(0.0);
//     }

//     @Override
//     public void periodic() {
//         // This method will be called once per scheduler run
//         SmartDashboard.putBoolean("Sensor", getSensorValue());
//     }
// }