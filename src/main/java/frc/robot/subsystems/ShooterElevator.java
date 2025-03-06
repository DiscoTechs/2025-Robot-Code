package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ShooterElevator extends SubsystemBase {
    final DigitalInput coralSensor;
    final DigitalInput sensor0;
    final DigitalInput sensor1;
    final DigitalInput sensor2;
    final DigitalInput sensor3;
    final DigitalInput sensor4;
    final DigitalInput sensor5;
    int currentLevel = 1;
    private final SparkMax rightMotor;
    private final SparkMax leftMotor;
    
    private final SparkMax intakeMotor;
    
    public ShooterElevator() { 
        sensor0 = new DigitalInput(Constants.ElavatorConstants.ELAVATOR_SENSOR_0);
        sensor1 = new DigitalInput(Constants.ElavatorConstants.ELAVATOR_SENSOR_1);
        System.out.println(sensor1);

        sensor2 = new DigitalInput(Constants.ElavatorConstants.ELAVATOR_SENSOR_2);
        sensor3 = new DigitalInput(Constants.ElavatorConstants.ELAVATOR_SENSOR_3);
        sensor4 = new DigitalInput(Constants.ElavatorConstants.ELAVATOR_SENSOR_4);
        sensor5 = new DigitalInput(Constants.ElavatorConstants.ELAVATOR_SENSOR_5);

        rightMotor = new SparkMax(Constants.ElavatorConstants.kRightElevatorMotorPort, MotorType.kBrushless);
        leftMotor = new SparkMax(Constants.ElavatorConstants.kLeftElevatorMotorPort, MotorType.kBrushless);

        //CORAL EFFECTOR
        coralSensor = new DigitalInput(Constants.ElavatorConstants.CORAL_SENSOR);
        intakeMotor = new SparkMax(Constants.CoralConstants.kCoralEffectorMotorPort, MotorType.kBrushless);
    }

    public boolean detectCoral() {
        return !coralSensor.get();
    }

    public void intakeSequence() {
        firstLevel();
        while (!detectCoral()) {
            outtake();
        }
        while (detectCoral()) {
            outtake();
            stopElevator();
        }
        stopShooter();
    }
    
    public boolean L1getSensorValue() {
        return !sensor1.get();
    }

    public boolean L2getSensorValue() {
        return !sensor2.get();
    }

    public boolean L3getSensorValue() {
        return !sensor3.get();
    }

    public boolean L4getSensorValue() {
        return !sensor4.get();
    }

    public boolean reachedLimit() {
        return !sensor5.get(); //change sign here after learning what sensor outputs
    }

    public int CurrentLevel() {
        return currentLevel;
    }

    public void moveUp(){
        rightMotor.set(0.5);
        leftMotor.set(-0.5);
    }

    public void moveDown(){
        rightMotor.set(-0.5);
        leftMotor.set(0.5);
    }

    public void firstLevel() {
        
        if(CurrentLevel() > 1){
            while(!L1getSensorValue()){
                moveDown();
            }
            currentLevel = 1;
        }
    }

    public void secondLevel() {
        if(CurrentLevel() > 2){
            while(!L2getSensorValue()){
                moveDown();
            }
            currentLevel = 2;
        }
        else if(CurrentLevel() < 2){
            while(!L2getSensorValue()){
                moveUp();
            }
            currentLevel = 2;
        }
    }

    public void thirdLevel() {
        if(CurrentLevel() > 3){
            while(!L3getSensorValue()){
                moveDown();
            }
            currentLevel = 3;
        }
        else if(CurrentLevel() < 3){
            while(!L3getSensorValue()){
                moveUp();
            }
            currentLevel = 3;
        }
    }

    public void fourthLevel() {
        if(CurrentLevel() < 4){
            while(!L4getSensorValue()){
                moveUp();
            }
            currentLevel = 4;
        }
    }

    public double speedSetter() {
        double shootingSpeed = 0.5;
        
        if (CurrentLevel() == 1) {
            shootingSpeed = Constants.CoralConstants.l1Speed;
        }
        else if (CurrentLevel() == 2){
            shootingSpeed = Constants.CoralConstants.l2speed;
        }
        else if (CurrentLevel() == 3) {
            shootingSpeed = Constants.CoralConstants.l3speed;
        }
        else if (CurrentLevel() == 4) {
            shootingSpeed = Constants.CoralConstants.l4speed;
        }
        else {
            shootingSpeed = Constants.CoralConstants.defaultSpeed;
        }
        return shootingSpeed;
    }

    public void intake() {
        intakeMotor.set(-speedSetter()); //update sign/speed accordingly
    }

    public void outtake() {
        intakeMotor.set(speedSetter()); //update sign/speed accordingly
    }

    public void stopElevator() {
        rightMotor.set(0.0);
        leftMotor.set(0.0);
    }

    public void stopShooter() {
        intakeMotor.set(0.0);
    }


    @Override
    public void periodic() {
        //System.out.println(L1getSensorValue());
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Sensor 1", L1getSensorValue());
        SmartDashboard.putBoolean("Sensor 2", L2getSensorValue());
        SmartDashboard.putBoolean("Sensor 3", L3getSensorValue());
        SmartDashboard.putBoolean("Sensor 4", L4getSensorValue());
    }
}