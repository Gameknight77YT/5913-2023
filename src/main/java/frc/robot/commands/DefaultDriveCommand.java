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

public class DefaultDriveCommand extends CommandBase {
    private Drivetrain m_drivetrainSubsystem;
    private Camera camera;

    private DoubleSupplier m_translationXSupplier;
    private DoubleSupplier m_translationYSupplier;
    private DoubleSupplier m_rotationSupplier;
    private BooleanSupplier isTrack;
    private BooleanSupplier isAutoBalance;

    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    private double error = 0, lasterror = 0, errorrate = 0;
    private PIDController pitchController = new PIDController(.0125, 0, 0);
    private PIDController rollController = new PIDController(.0125, 0, 0);

    public DefaultDriveCommand(Drivetrain drivetrainSubsystem,
                               Camera camera,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               BooleanSupplier isTrack,
                               BooleanSupplier isAutoBalance) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.camera = camera;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.isTrack = isTrack;
        this.isAutoBalance = isAutoBalance;

        this.xLimiter = new SlewRateLimiter(Constants.MAX_VELOCITY_METERS_PER_SECOND/.5);
        this.yLimiter = new SlewRateLimiter(Constants.MAX_VELOCITY_METERS_PER_SECOND/.5);
        this.turningLimiter = new SlewRateLimiter(Constants.MAX_VELOCITY_METERS_PER_SECOND/.125);

        addRequirements(drivetrainSubsystem);

        pitchController.setTolerance(3);
        rollController.setTolerance(3);
        
    }

    @Override
    public void execute() {
        
        //https://pdocs.kauailabs.com/navx-mxp/examples/automatic-balancing/
        //bad example, not working

        
        double rotationRate = m_rotationSupplier.getAsDouble();
        double xAxisRate            = m_translationXSupplier.getAsDouble();
        double yAxisRate            = m_translationYSupplier.getAsDouble();
        double pitchAngleDegrees    = m_drivetrainSubsystem.getPitch();
        double rollAngleDegrees     = m_drivetrainSubsystem.getRoll();

        xAxisRate = xLimiter.calculate(xAxisRate);
        yAxisRate = yLimiter.calculate(yAxisRate);
        rotationRate = turningLimiter.calculate(rotationRate);
        
        if(isAutoBalance.getAsBoolean()){
            
        
            // Control drive system automatically, 
            // driving in reverse direction of pitch/roll angle,
            // with a magnitude based upon the angle
            errorrate = lasterror - error;
           
           if(errorrate > .25){

           }else{
            yAxisRate += pitchController.calculate(pitchAngleDegrees, 0);
            xAxisRate += rollController.calculate(rollAngleDegrees, 0);
           }
            
            lasterror = error;
        }

        
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xAxisRate,
            yAxisRate,
            rotationRate,
            m_drivetrainSubsystem.getGyroscopeRotation());
        if(isTrack.getAsBoolean()){
            speeds.vyMetersPerSecond += camera.getMoveInput();
        }
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.setModuleStates(Constants.kinematics.toSwerveModuleStates(speeds)
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stopDrive();
    }
}
