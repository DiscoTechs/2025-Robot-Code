package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SensorSubsystem extends SubsystemBase {
    final DigitalInput sensor;

    public SensorSubsystem() {
        sensor = new DigitalInput(Constants.CoralConstants.CORAL_SENSOR);  
    }

    public boolean getSensorValue() {
        return !sensor.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putBoolean("Sensor", getSensorValue());
    }
}