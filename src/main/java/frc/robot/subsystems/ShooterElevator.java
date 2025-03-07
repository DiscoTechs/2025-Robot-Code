package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ShooterElevator extends SubsystemBase {
    public DigitalInput coralSensor;
    public final SparkMax shooterMotor;
    public final DigitalInput Level1Sensor;
    public final DigitalInput Level2Sensor;
    public final DigitalInput Level3Sensor;
    public final DigitalInput Level4Sensor;
    public final DigitalInput LimitSensor;
    public final SparkMax rightElevatorMotor;
    public final SparkMax leftElevatorMotor;
    
    public ShooterElevator() { 
        //CORAL EFFECTOR
        coralSensor = new DigitalInput(Constants.ElavatorConstants.CORAL_SENSOR);
        shooterMotor = new SparkMax(Constants.CoralConstants.kCoralEffectorMotorPort, MotorType.kBrushless);

        //ELEVATOR
        Level1Sensor = new DigitalInput(Constants.ElavatorConstants.ELAVATOR_SENSOR_1);
        Level2Sensor = new DigitalInput(Constants.ElavatorConstants.ELAVATOR_SENSOR_2);
        Level3Sensor = new DigitalInput(Constants.ElavatorConstants.ELAVATOR_SENSOR_3);
        Level4Sensor = new DigitalInput(Constants.ElavatorConstants.ELAVATOR_SENSOR_4);
        LimitSensor = new DigitalInput(Constants.ElavatorConstants.ELAVATOR_LIMIT_SENSOR);

        rightElevatorMotor = new SparkMax(Constants.ElavatorConstants.kRightElevatorMotorPort, MotorType.kBrushless);
        leftElevatorMotor = new SparkMax(Constants.ElavatorConstants.kLeftElevatorMotorPort, MotorType.kBrushless);  
    }

    public void goToFirstLevel() {
        stopElevatorIfLimitReached();
        if (isSensorDetected(Level1Sensor)) {
            stopElevator();
        }
        
        else {
            while (!isSensorDetected(Level1Sensor)) {
                moveElevatorDown();
            }
            stopElevator();
        }
    }

    public void goToSecondLevel() {
        //goToFirstLevel();
        
        while (!isSensorDetected(Level2Sensor)) {
            if (isTopLimitReached()) {
                stopElevator();
                break; //where does this BREAK break out of?
            }
            else {
                moveElevatorUp();
            }  
        }
        stopElevator();
    }

    public void goToThirdLevel() {
        //goToFirstLevel();

        while (!isSensorDetected(Level3Sensor)) {
            if (isTopLimitReached()) {
                stopElevator();
                break; //where does this BREAK break out of?
            }
            else {
                moveElevatorUp();
            }  
        }
        stopElevator();
    }

    public void goToFourthLevel() {
        //goToFirstLevel();

        while (!isSensorDetected(Level4Sensor)) {
            if (isTopLimitReached()) {
                stopElevator();
                break; //where does this BREAK break out of?
            }
            else {
                moveElevatorUp();
            }  
        }
        stopElevator();
    }

    public void moveElevatorUp() {
        leftElevatorMotor.set(-0.4);
        rightElevatorMotor.set(0.4);
    }

    public void moveElevatorDown() {
        leftElevatorMotor.set(0.4);
        rightElevatorMotor.set(-0.4);
    }

    public void stopElevator() {
        leftElevatorMotor.set(0);
        rightElevatorMotor.set(0);
    }

    public void intakeCoral() {
        shooterMotor.set(-0.5);
    }

    public void outtakeCoral() {
        shooterMotor.set(0.5);
    }

    public void stopShooter() {
        shooterMotor.set(0.0);
    }

    public boolean isSensorDetected(DigitalInput x) {
        return !x.get();
    }

    public boolean isTopLimitReached() {
        if (isSensorDetected(LimitSensor)) {
            return true;
        }
        else {
            return false;
        }
    }
    //maybe delete stopElevatorIfLimitReached()
    public void stopElevatorIfLimitReached() {
        if (isTopLimitReached()) {
            stopElevator();
        }
    }
    //intakeSequence() is purely for auto
    public void intakeSequence() {
        goToFirstLevel();
        while (!isSensorDetected(coralSensor)) {
            outtakeCoral();
        }
        while (isSensorDetected(coralSensor)) {
            outtakeCoral();
            stopElevator();
        }
        stopShooter();
    }

    @Override
    public void periodic() {
        //System.out.println(L1getSensorValue());
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Sensor 1", isSensorDetected(Level1Sensor));
        SmartDashboard.putBoolean("Sensor 2", isSensorDetected(Level2Sensor));
        SmartDashboard.putBoolean("Sensor 3", isSensorDetected(Level3Sensor));
        SmartDashboard.putBoolean("Sensor 4", isSensorDetected(Level4Sensor));
    }
}