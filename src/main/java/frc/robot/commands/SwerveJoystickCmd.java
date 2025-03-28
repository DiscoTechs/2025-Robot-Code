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

        boolean lefttTriggerPressed = false;
        if (driverJoystick.getRawAxis(Constants.OIConstants.XBX_L_TRIG) > 0.1) {
            lefttTriggerPressed = true;
        }
        
        if (lefttTriggerPressed) {
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
        if (driverJoystick.getRawAxis(Constants.OIConstants.XBX_R_TRIG) > 0.1) {
            rightTriggerPressed = true;
        }

        //IF USING LIMELIGHT/IF LIMELIGHT IS PLUGGED IN AND WORKING
    
        if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getCurrentPipelineIndex("limelight") == 0 && rightTriggerPressed) {
    
            chassisSpeeds = limelightFollow();
            discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        }
        else if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getCurrentPipelineIndex("limelight") == 1 && rightTriggerPressed) {
            
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            chassisSpeeds = limelightFollow();
            discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        }

        else if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getCurrentPipelineIndex("limelight") == 0 && (driverJoystick.getPOV() == 270)) {
            //assumed that tx is currently very close to or equal to zero, that is, after we center align it with right trigger

            //move left if pressed 270 (left pov)
            if (tx < Constants.OIConstants.DISTANCE_FROM_CENTER) { //if this sign is positive, all the signs for the constant are positive below, including in the else statement, in Math.abs()
                if (Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 10) {
                    ySpeed = -0.3;
                }
                else if ((Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 10) && (Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 8)) {
                    ySpeed = -0.3 * 0.8;
                }
                else if ((Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 8) && (Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 6)) {
                    ySpeed = -0.3 * 0.65;
                }
                else if ((Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 6) && (Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 4)) {
                    ySpeed = -0.3 * 0.5;
                }
                else if ((Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 4) && (Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 2)) {
                    ySpeed = -0.3 * 0.40;
                }
                else if ((Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 2) && (Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 0.3)) {
                    ySpeed = -0.3 * 0.25;
                }
                else {
                    ySpeed = 0;
                }

                //comment out code above (and else statment below) and unncoment code below if it doesn't work
                //ySpeed = -0.3
            }

            //if overshoot by acccident, move in opposite direction
            else {
                if (Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 10) {
                    ySpeed = 0.3;
                }
                else if ((Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 10) && (Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 8)) {
                    ySpeed = 0.3 * 0.8;
                }
                else if ((Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 8) && (Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 6)) {
                    ySpeed = 0.3 * 0.65;
                }
                else if ((Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 6) && (Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 4)) {
                    ySpeed = 0.3 * 0.5;
                }
                else if ((Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 4) && (Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 2)) {
                    ySpeed = 0.3 * 0.40;
                }
                else if ((Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 2) && (Math.abs(Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 0.3)) {
                    ySpeed = 0.3 * 0.25;
                }
                else {
                    ySpeed = 0;
                }
            }
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        }

        else if (LimelightHelpers.getTV("limelight") && LimelightHelpers.getCurrentPipelineIndex("limelight") == 0 && (driverJoystick.getPOV() == 90)) {
            //assumed that tx is currently very close to or equal to zero, that is, after we center align it with right trigger

            //move right if pressed 90 pov (right pov)
            if (tx > -Constants.OIConstants.DISTANCE_FROM_CENTER) { //if this sign is negative, all the signs for the constant are negtive below, including in the else statement, in Math.abs()
                if (Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 10) {
                    ySpeed = 0.3;
                }
                else if ((Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 10) && (Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 8)) {
                    ySpeed = 0.3 * 0.8;
                }
                else if ((Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 8) && (Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 6)) {
                    ySpeed = 0.3 * 0.65;
                }
                else if ((Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 6) && (Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 4)) {
                    ySpeed = 0.3 * 0.50;
                }
                else if ((Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 4) && (Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 2)) {
                    ySpeed = 0.3 * 0.40;
                }
                else if ((Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 2) && (Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 0.3)) {
                    ySpeed = 0.3 * 0.25;
                }
                else {
                    ySpeed = 0;
                }

                //comment out code above (and else statment below) and unncoment code below if it doesn't work
                //ySpeed = -0.3
            }

            //if overshoot by acccident, move in opposite direction
            else {
                if (Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 10) {
                    ySpeed = -0.3;
                }
                else if ((Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 10) && (Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 8)) {
                    ySpeed = -0.3 * 0.8;
                }
                else if ((Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 8) && (Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 6)) {
                    ySpeed = -0.3 * 0.65;
                }
                else if ((Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 6) && (Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 4)) {
                    ySpeed = -0.3 * 0.5;
                }
                else if ((Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 4) && (Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 2)) {
                    ySpeed = -0.3 * 0.40;
                }
                else if ((Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) < 2) && (Math.abs(-Constants.OIConstants.DISTANCE_FROM_CENTER - tx) >= 0.3)) {
                    ySpeed = -0.3 * 0.25;
                }
                else {
                    ySpeed = 0;
                }
            }
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
            discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        }


        //IF NOT USING LIMELIGHT
        else {
            //GO TO POSITION 1 CORRECT ANGLE
            // if (driverJoystick.getPOV() == Constants.OIConstants.LEFT_POV) { // the button at the top button to face the wall
            //     turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.POSITION_1_ANGLE);
            //     System.out.println(swerveSubsystem.getAngle());
            // }

            // //GO TO POSITION 2 CORRECT ANGLE
            // else if (driverJoystick.getPOV() == Constants.OIConstants.UP_POV) { // the button at the top button to face the wall
            //     turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.POSITION_2_ANGLE);
            //     System.out.println(swerveSubsystem.getAngle());
            // }

            // //GO TO POSITION 3 CORRECT ANGLE
            // else if (driverJoystick.getPOV() == Constants.OIConstants.RIGHT_POV) { // the button at the top button to face the wall
            //     turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.POSITION_3_ANGLE);
            //     System.out.println(swerveSubsystem.getAngle());
            // }

            // //GO TO POSITION 4 CORRECT ANGLE
            // else if (driverJoystick.getPOV() == Constants.OIConstants.DOWN_POV) { // the button at the top button to face the wall
            //     turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.POSITION_4_ANGLE);
            //     System.out.println(swerveSubsystem.getAngle());
            // }

            // //GO TO FACE FORWARD CORRECT ANGLE
            // else if (driverJoystick.getRawButton(Constants.OIConstants.XBX_Y)) { // the button at the top button to face the wall
            //     turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.FACE_FORWARD_ANGLE);
            //     System.out.println(swerveSubsystem.getAngle());
            // } 
            // //GO TO FACE BACKWARD CORRECT ANGLE
            // else if (driverJoystick.getRawButton(Constants.OIConstants.XBX_A)) { // the button at the top, or 'y', button to face the wall
            //     turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.FACE_BACKWARDS_ANGLE);
            //     System.out.println(swerveSubsystem.getAngle());
            // }
            // //GO TO FACE LEFT CORRECT ANGLE
            // else if (driverJoystick.getRawButton(Constants.OIConstants.XBX_X)) { // the button at the top, or 'y', button to face the wall
            //     turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.FACE_LEFT_ANGLE);
            //     System.out.println(swerveSubsystem.getAngle());
            // }
            // //GO TO FACE RIGHT CORRECT ANGLE
            // else if (driverJoystick.getRawButton(Constants.OIConstants.XBX_B)) { // the button at the top, or 'y', button to face the wall
            //     turningSpeed = swerveSubsystem.getTurningSpeed(Constants.OIConstants.FACE_RIGHT_ANGLE);
            //     System.out.println(swerveSubsystem.getAngle());
            // }
            
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

    public ChassisSpeeds limelightFollow_old(double xSpeed, double ySpeed, double turningSpeed) {

         //LimelightHelpers.setPipelineIndex("limelight", 1);

         tx = LimelightHelpers.getTX("limelight");
         double ta = LimelightHelpers.getTX("limelight");
         
         //System.out.println(tx);
         //MOVE ROBOT LEFT AND RIGHT IF ALGAE IS NOT IN THE CENTER OF LIMELIGHT VIEW
         if (tx < -7) {
             ySpeed = 0.5; // -1.0 / limelightTA / 3;
         } else if (tx > -7) {
             ySpeed = -0.5; // -1.0 / limelightTA / 3;
         }
         else {
             ySpeed = 0;
         }

        return new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }

    double ta = 0;

    public ChassisSpeeds limelightFollow(double xSpeed, double ySpeed, double turningSpeed) {

        //LimelightHelpers.setPipelineIndex("limelight", 1);

        tx = LimelightHelpers.getTX("limelight");
        ta = LimelightHelpers.getTA("limelight");

        double[] botPose = LimelightHelpers.getBotPose_TargetSpace("limelight");

        // Angle of the april tag
        double offsetAngle = botPose[4];

        // XSPEED
        xSpeed = 1.0 - ta / 100 * 1.1;  //110% of proportional value

        // YSPEED - slide left/right based on offset
        if(Math.abs(offsetAngle) > 5.0) {
            ySpeed = -(offsetAngle / 90) * 1.5; // 150% of proportional speed
        } else {
            ySpeed = 0;
        }

        // TURNING SPEED
        if (Math.abs(tx) < 1) {tx = 0;} // lame deadband code
        turningSpeed = -tx * 0.06;      // trial and error, kP

       return new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
   }

   // helper method so we don't have to send x, y, and turn
   public ChassisSpeeds limelightFollow() {
    return limelightFollow(0, 0, 0);
   }
}