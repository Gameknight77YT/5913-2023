package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private BooleanSupplier isAutoBalance;

    private boolean autoBalanceXMode = false;
    private boolean autoBalanceYMode = false;

    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public DefaultDriveCommand(Drivetrain drivetrainSubsystem,
                               Camera camera,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               BooleanSupplier isAutoBalance) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.camera = camera;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.isAutoBalance = isAutoBalance;

        this.xLimiter = new SlewRateLimiter(Constants.MAX_VELOCITY_METERS_PER_SECOND/.5);
        this.yLimiter = new SlewRateLimiter(Constants.MAX_VELOCITY_METERS_PER_SECOND/.5);
        this.turningLimiter = new SlewRateLimiter(.1);

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        
        //https://pdocs.kauailabs.com/navx-mxp/examples/automatic-balancing/
        double kOffBalanceAngleThresholdDegrees = 10;
        double kOonBalanceAngleThresholdDegrees  = 5;

        //auto balance
        double rotationRate = m_rotationSupplier.getAsDouble();
        double xAxisRate            = m_translationXSupplier.getAsDouble();
        double yAxisRate            = m_translationYSupplier.getAsDouble();
        double pitchAngleDegrees    = m_drivetrainSubsystem.getPitch();
        double rollAngleDegrees     = m_drivetrainSubsystem.getRoll();

        xAxisRate = xLimiter.calculate(xAxisRate);
        yAxisRate = yLimiter.calculate(yAxisRate);
        //rotationRate = turningLimiter.calculate(rollAngleDegrees);
        
        /*if(isAutoBalance.getAsBoolean()){
            if ( !autoBalanceXMode && 
             (Math.abs(pitchAngleDegrees) >= 
              Math.abs(kOffBalanceAngleThresholdDegrees))) {
            autoBalanceXMode = true;
            }
            else if ( autoBalanceXMode && 
                  (Math.abs(pitchAngleDegrees) <= 
                   Math.abs(kOonBalanceAngleThresholdDegrees))) {
            autoBalanceXMode = false;
            }
            if ( !autoBalanceYMode && 
             (Math.abs(pitchAngleDegrees) >= 
              Math.abs(kOffBalanceAngleThresholdDegrees))) {
            autoBalanceYMode = true;
            }  
            else if ( autoBalanceYMode && 
                  (Math.abs(pitchAngleDegrees) <= 
                   Math.abs(kOonBalanceAngleThresholdDegrees))) {
            autoBalanceYMode = false;
            }
        
            // Control drive system automatically, 
            // driving in reverse direction of pitch/roll angle,
            // with a magnitude based upon the angle
        
            if ( autoBalanceXMode ) {
                double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
                xAxisRate = Math.sin(pitchAngleRadians) * -1;
            }
            if ( autoBalanceYMode ) {
                double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
                yAxisRate = Math.sin(rollAngleRadians) * -1;
            }
        }*/

        if(isAutoBalance.getAsBoolean()){
            yAxisRate += -camera.getMoveInput(); 
        }
        
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        m_drivetrainSubsystem.setModuleStates(Constants.kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xAxisRate,
                    yAxisRate,
                    rotationRate,
                    m_drivetrainSubsystem.getGyroscopeRotation()
            ))
        );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.stopDrive();
    }
}
