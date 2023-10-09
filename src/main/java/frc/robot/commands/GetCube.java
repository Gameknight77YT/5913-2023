package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class GetCube extends CommandBase {
    private Drivetrain m_drivetrainSubsystem;
    private Camera camera;

    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    private double error = 0, lasterror = 0, errorrate = 0;
    

    public GetCube(Drivetrain drivetrainSubsystem, Camera camera) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.camera = camera;

        this.xLimiter = new SlewRateLimiter(Constants.MAX_VELOCITY_METERS_PER_SECOND/.5);
        this.yLimiter = new SlewRateLimiter(Constants.MAX_VELOCITY_METERS_PER_SECOND/.5);
        this.turningLimiter = new SlewRateLimiter(Constants.MAX_VELOCITY_METERS_PER_SECOND/.125);

        addRequirements(drivetrainSubsystem);
        
    }

    @Override
    public void execute() {

        
        double xAxisRate            = 0;
        double yAxisRate            = 0;
        double pitchAngleDegrees    = m_drivetrainSubsystem.getPitch();
        double rollAngleDegrees     = m_drivetrainSubsystem.getRoll();

        xAxisRate = xLimiter.calculate(xAxisRate);
        yAxisRate = yLimiter.calculate(yAxisRate);
        
        

        
        
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.setModuleStates(Constants.kinematics.toSwerveModuleStates(
                new ChassisSpeeds(
                    1,
                    camera.getMoveInput(),
                    0
                )
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stopDrive();
    }
}
