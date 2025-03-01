package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Elevator extends SubsystemBase {
    final DigitalInput sensor1;
    final DigitalInput sensor2;
    final DigitalInput sensor3;
    final DigitalInput sensor4;
    int currentLevel = 1;
    private final SparkMax rightMotor;
    private final SparkMax leftMotor;
    
    public Elevator() {
        sensor1 = new DigitalInput(Constants.ElavatorConstants.ELAVATOR_SENSOR_1);
        System.out.println(sensor1);

        sensor2 = new DigitalInput(Constants.ElavatorConstants.ELAVATOR_SENSOR_2);
        sensor3 = new DigitalInput(Constants.ElavatorConstants.ELAVATOR_SENSOR_3);
        sensor4 = new DigitalInput(Constants.ElavatorConstants.ELAVATOR_SENSOR_4);

        rightMotor = new SparkMax(Constants.ElavatorConstants.kRightElevatorMotorPort, MotorType.kBrushless);
        leftMotor = new SparkMax(Constants.ElavatorConstants.kLeftElevatorMotorPort, MotorType.kBrushless);
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

    public void stop() {
        rightMotor.set(0.0);
        leftMotor.set(0.0);
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