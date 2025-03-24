package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction, resetHeading;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final Joystick driverJoystick;
    private double tx;
    private double kP = 0.05; // change value if needed
    private int turningDirection;
    private double angle;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> resetHeading, Joystick driverJoystick) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.resetHeading = resetHeading;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        this.driverJoystick = driverJoystick;

        LimelightHelpers.setPipelineIndex("limelight", 0);
        
        addRequirements(swerveSubsystem);
    }
    @Override
    public void initialize() {}
    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();
        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        ChassisSpeeds discreteSpeeds;
        Pose3d pose = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
        // System.out.println("TARGET ROTATION: " + (pose.getRotation().getQuaternion().getY())*180/Math.PI); //rotation calc

        boolean leftTriggerPressed = false;
        if (driverJoystick.getRawAxis(Constants.OIConstants.XBX_L_TRIG) > 0.1) {
            leftTriggerPressed = true;
        }
        
        if (leftTriggerPressed) {
            if (LimelightHelpers.getCurrentPipelineIndex("limelight") == 0) {
                LimelightHelpers.setPipelineIndex("limelight", 1);
            }
            else if (LimelightHelpers.getCurrentPipelineIndex("limelight") == 1) {
                LimelightHelpers.setPipelineIndex("limelight", 0);
            }
            SmartDashboard.putNumber("Current Pipeline", (LimelightHelpers.getCurrentPipelineIndex("limelight")));
        }

        

        //test for getCurrentPipelineIndex()

        //System.out.println(LimelightHelpers.getCurrentPipelineIndex("limelight"));

        //test for current drive angle

        boolean rightTriggerPressed = false;
        boolean angleButtonPressed = false;
        if (driverJoystick.getRawAxis(Constants.OIConstants.XBX_R_TRIG) > 0.1) {
            rightTriggerPressed = true;
        }

        if ((driverJoystick.getPOV() == Constants.OIConstants.LEFT_POV) ||
        (driverJoystick.getPOV() == Constants.OIConstants.RIGHT_POV) ||
        (driverJoystick.getPOV() == Constants.OIConstants.UP_POV) || 
        (driverJoystick.getPOV() == Constants.OIConstants.DOWN_POV) ||
        (driverJoystick.getPOV() == Constants.OIConstants.XBX_A) ||
        (driverJoystick.getPOV() == Constants.OIConstants.XBX_B) ||
        (driverJoystick.getPOV() == Constants.OIConstants.XBX_X) ||
        (driverJoystick.getPOV() == Constants.OIConstants.XBX_Y)) {
            angleButtonPressed = true;
        }

        //IF USING LIMELIGHT/IF LIMELIGHT IS PLUGGED IN AND WORKING
    
        //For REEF april tag horizontal alignment
        if (LimelightHelpers.getCurrentPipelineIndex("limelight") == 0 && 
            (rightTriggerPressed || angleButtonPressed)) {
            
            tx = LimelightHelpers.getTX("limelight");
            double ta = LimelightHelpers.getTX("limelight");
            
            //System.out.println(tx);
            //MOVE ROBOT LEFT AND RIGHT IF ALGAE IS NOT IN THE CENTER OF LIMELIGHT VIEW
            // if (tx < -3) {
            //     ySpeed = 0.3; // -1.0 / limelightTA / 3;
            // } else if (tx > 3) {
            //     ySpeed = -0.3; // -1.0 / limelightTA / 3;
            // }
            // else {
            //     ySpeed = 0;
            // }

            if (driverJoystick.getPOV() == Constants.OIConstants.LEFT_POV) { // the button at the top button to face the wall
                turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.POSITION_1_ANGLE);
                System.out.println(swerveSubsystem.getAngle());
            }

            //GO TO POSITION 2 CORRECT ANGLE
            else if (driverJoystick.getPOV() == Constants.OIConstants.UP_POV) { // the button at the top button to face the wall
                turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.POSITION_2_ANGLE);
                System.out.println(swerveSubsystem.getAngle());
            }

            //GO TO POSITION 3 CORRECT ANGLE
            else if (driverJoystick.getPOV() == Constants.OIConstants.RIGHT_POV) { // the button at the top button to face the wall
                turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.POSITION_3_ANGLE);
                System.out.println(swerveSubsystem.getAngle());
            }

            //GO TO POSITION 4 CORRECT ANGLE
            else if (driverJoystick.getPOV() == Constants.OIConstants.DOWN_POV) { // the button at the top button to face the wall
                turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.POSITION_4_ANGLE);
                System.out.println(swerveSubsystem.getAngle());
            }

            //GO TO FACE FORWARD CORRECT ANGLE
            else if (driverJoystick.getRawButton(Constants.OIConstants.XBX_Y)) { // the button at the top button to face the wall
                turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.FACE_FORWARD_ANGLE);
                System.out.println(swerveSubsystem.getAngle());
            } 
            //GO TO FACE BACKWARD CORRECT ANGLE
            else if (driverJoystick.getRawButton(Constants.OIConstants.XBX_A)) { // the button at the top, or 'y', button to face the wall
                turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.FACE_BACKWARDS_ANGLE);
                System.out.println(swerveSubsystem.getAngle());
            }

            ySpeed = swerveSubsystem.getYSpeed(tx);
            if (ta < 0.2) {
                xSpeed = 1;
            }
            else {
                xSpeed = 0.25;
            }

            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        }
        
        //For FLOOR ALGAE april tag horizontal alignment
        else if (LimelightHelpers.getCurrentPipelineIndex("limelight") == 1 && 
            (rightTriggerPressed || angleButtonPressed)) {
            
            tx = LimelightHelpers.getTX("limelight");
            double ta = LimelightHelpers.getTA("limelight");
            
            //System.out.println(tx);
            //MOVE ROBOT LEFT AND RIGHT IF ALGAE IS NOT IN THE CENTER OF LIMELIGHT VIEW
            // if (tx < -7) {
            //     ySpeed = 0.5; // -1.0 / limelightTA / 3;
            // } else if (tx > -7) {
            //     ySpeed = -0.5; // -1.0 / limelightTA / 3;
            // }
            // else {
            //     ySpeed = 0;
            // }
        
            // xSpeed = 0.5;

            //GO TO FACE FORWARD CORRECT ANGLE
            if (driverJoystick.getRawButton(Constants.OIConstants.XBX_Y)) { // the button at the top button to face the wall
                turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.FACE_FORWARD_ANGLE);
                System.out.println(swerveSubsystem.getAngle());
            } 
            //GO TO FACE BACKWARD CORRECT ANGLE
            else if (driverJoystick.getRawButton(Constants.OIConstants.XBX_A)) { // the button at the top, or 'y', button to face the wall
                turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.FACE_BACKWARDS_ANGLE);
                System.out.println(swerveSubsystem.getAngle());
            }
            //GO TO FACE LEFT CORRECT ANGLE
            else if (driverJoystick.getRawButton(Constants.OIConstants.XBX_X)) { // the button at the top, or 'y', button to face the wall
                turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.FACE_LEFT_ANGLE);
                System.out.println(swerveSubsystem.getAngle());
            }
            //GO TO FACE RIGHT CORRECT ANGLE
            else if (driverJoystick.getRawButton(Constants.OIConstants.XBX_B)) { // the button at the top, or 'y', button to face the wall
                turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.FACE_RIGHT_ANGLE);
                System.out.println(swerveSubsystem.getAngle());
            }

            ySpeed = swerveSubsystem.getYSpeed(tx);

            if (ta < 0.2) {
                xSpeed = 1;
            }
            else {
                xSpeed = 0.25;
            }
            
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        }
        //IF NOT USING LIMELIGHT
        else {
            //Adjust left
            if (driverJoystick.getRawButton(Constants.OIConstants.ADJUST_LEFT)) {
                ySpeed = 0.3; //adjust value to exactly go into coral 
            }
            //Adjust right
            else if (driverJoystick.getRawButton(Constants.OIConstants.ADJUST_RIGHT)) {
                ySpeed = -0.3; //adjust value to exactly go into coral
            }
            
            if (fieldOrientedFunction.get()) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
                discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
            }
            else { //if robot oriented
                chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
                discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
            }
        }

        if (resetHeading.get()) {                                                                                                                                                                       
            swerveSubsystem.resetGyro();
        }
        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stop();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}