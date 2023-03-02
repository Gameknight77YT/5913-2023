// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.FRONT_RIGHT_MODULE_STEER_OFFSET;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  

  

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  //  Remove if you are using a Pigeon
  //private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
  //  Uncomment if you are using a NavX
  private final AHRS navx = new AHRS(Port.kMXP, (byte) 200); // NavX connected over MXP

  private Pose2d pose = new Pose2d();

   // Odometry class for tracking robot pose
   SwerveDriveOdometry odometry;

   //private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
   private SwerveModuleState[] states = Constants.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

   private SwerveModule[] modules;

  // These are our modules. We initialize them in the constructor.
  private SwerveModule frontLeftModule;
  private SwerveModule frontRightModule;
  private SwerveModule backLeftModule;
  private SwerveModule backRightModule;

  // Create a new SimpleMotorFeedforward with gains kS, kV, and kA
  private SimpleMotorFeedforward feedforwardRight = new SimpleMotorFeedforward(Constants.kSRight, Constants.kVRight, Constants.kARight);
  private SimpleMotorFeedforward feedforwardLeft = new SimpleMotorFeedforward(Constants.kSLeft, Constants.kVLeft, Constants.kALeft);

  private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

  public Drivetrain() {
        new Thread(() -> {
                try {
                    Thread.sleep(1000);
                    zeroGyroscope();
                } catch (Exception e) {
                }
            }).start();
        
    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.
    // Setup motor configuration
    frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            
            Mk4SwerveModuleHelper.GearRatio.L1,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4SwerveModuleHelper.GearRatio.L1,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4SwerveModuleHelper.GearRatio.L1,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4SwerveModuleHelper.GearRatio.L1,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );

    modules = new SwerveModule[] {frontLeftModule, frontRightModule, backLeftModule, backRightModule};
     
    odometry =  
        new SwerveDriveOdometry(Constants.kinematics, getGyroscopeRotation(), getSwerveModulePositions());
    
        
  }

  public SwerveModulePosition[] getSwerveModulePositions(){
        SwerveModulePosition[] swerveModulePositions = {
                modules[0].getSwerveModulePosition(),
                modules[1].getSwerveModulePosition(),
                modules[2].getSwerveModulePosition(),
                modules[3].getSwerveModulePosition()
            };
        return swerveModulePositions;
  } 


  

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    //  Remove if you are using a Pigeon
   // m_pigeon.setFusedHeading(0.0);

    //  Uncomment if you are using a NavX
    navx.reset();
    
  }

  

  public Rotation2d getGyroscopeRotation() {
    //  Remove if you are using a Pigeon
    //return Rotation2d.fromDegrees(m_pigeon.getFusedHeading());

    //  Uncomment if you are using a NavX
    //if (navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      //return Rotation2d.fromDegrees(navx.getFusedHeading());
    //}

    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return navx.getRotation2d();//Rotation2d.fromDegrees(Math.IEEEremainder(navx.getAngle(), 360));//Rotation2d.fromDegrees(360.0 - navx.getYaw());
  }

  public double getPitch(){
        return navx.getPitch();
  }

  public double getRoll(){
        return navx.getRoll();
  }

  

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), pose);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param inputStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] inputStates) {
        states = inputStates;
        
        
  }

  public void stopDrive(){
        states = Constants.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
  }

  private void updateOdometry() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            var module = modules[i];

            moduleStates[i] = new SwerveModuleState(module.getDriveVelocity(), new Rotation2d(module.getSteerAngle()));
        }
        //states = moduleStates;
        pose = odometry.update(getGyroscopeRotation(), getSwerveModulePositions());
        
    }


  @Override
  public void periodic() {
        
        updateOdometry();
    
        SmartDashboard.putNumber("gyro Heading", getGyroscopeRotation().getDegrees());
        SmartDashboard.putNumber("Robot Heading", getPose().getRotation().getDegrees());
        SmartDashboard.putString("Robot Location", getPose().getX() + ", " + getPose().getY());
        
        SwerveDriveKinematics.desaturateWheelSpeeds(
                states, Constants.MAX_VELOCITY_METERS_PER_SECOND);

        //frontLeftModule.set(states[0].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE, states[0].angle.getRadians());
        //frontRightModule.set(states[1].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE, states[1].angle.getRadians());
        //backLeftModule.set(states[2].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE, states[2].angle.getRadians());
        //backRightModule.set(states[3].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE, states[3].angle.getRadians());

        frontLeftModule.set(feedforwardLeft.calculate(states[0].speedMetersPerSecond), states[0].angle.getRadians());
        frontRightModule.set(feedforwardRight.calculate(states[1].speedMetersPerSecond), states[1].angle.getRadians());
        backLeftModule.set(feedforwardLeft.calculate(states[2].speedMetersPerSecond), states[2].angle.getRadians());
        backRightModule.set(feedforwardRight.calculate(states[3].speedMetersPerSecond), states[3].angle.getRadians());
                
  }


  
}
