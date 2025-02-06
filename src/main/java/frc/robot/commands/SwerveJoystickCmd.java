package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction, resetHeading, visionTurn;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final Joystick driverJoystick;
    private double tx;
    private double kP = 0.05; // change value if needed

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Boolean> resetHeading, Supplier<Boolean> visionTurn, Joystick driverJoystick) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.resetHeading = resetHeading;
        this.visionTurn = visionTurn;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        this.driverJoystick = driverJoystick;
        
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
        ChassisSpeeds discreteSpeeds; // remove the drift yo

        if (this.visionTurn.get()) {
            LimelightHelpers.setPipelineIndex("limelight", 2);
        }

            if (this.visionTurn.get() && LimelightHelpers.getTV("limelight")) {
                double aprilTagID = LimelightHelpers.getLimelightNTDouble("limelight", "tid");
                if (aprilTagID == RobotContainer.limelightFilterChooser.getSelected()) {
                    tx = LimelightHelpers.getTX("limelight");

                    xSpeed = 0;
                    ySpeed = 0;
                    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
                    turningSpeed = turningLimiter.calculate(kP * tx) * 2;

                    turningSpeed = -tx * .12;

                    if (LimelightHelpers.getTA("limelight") < 18) {
                        xSpeed = -.6; // -1.0 / LimelightHelpers.getTA("limelight") / 3;
                    }

                    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

                    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
                    discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
            } else {
                if (fieldOrientedFunction.get()) {
                    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
                    discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
                } else {
                    chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
                    discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
                }
            }
        } else {
            if (fieldOrientedFunction.get()) {
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
                discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
            } else {
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