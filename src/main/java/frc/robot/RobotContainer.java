// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();

  private final XboxController controller = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    drivetrain.setDefaultCommand(new DefaultDriveCommand(
            drivetrain,
            () -> -modifyAxis(controller.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(controller.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(controller.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //  button zeros the gyroscope
    new Trigger(controller::getRightBumper)
            .onTrue( new InstantCommand(() -> drivetrain.zeroGyroscope()));
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // This will load the file "Example Path.path" and generate it with a max velocity of 8 m/s and a max acceleration of 5 m/s^2
    PathPlannerTrajectory examplePath = PathPlanner.loadPath(
      "Test3",
      Constants.MAX_VELOCITY_METERS_PER_SECOND,
      Constants.MAX_acceleration_METERS_PER_SECOND);

      HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("Marker1", new PrintCommand("Passed marker 1"));
      eventMap.put("Stop", new InstantCommand(() -> drivetrain.stopDrive(), drivetrain));

      SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        drivetrain::getPose, 
        drivetrain::resetOdometry, 
        Constants.kinematics, 
        new PIDConstants(Constants.kPXController, 0, 0), 
        new PIDConstants(Constants.kPThetaController, 0, Constants.kDThetaController), 
        drivetrain::setModuleStates, 
        eventMap, 
        drivetrain
        );

        Command auto = autoBuilder.fullAuto(examplePath);

      
      PPSwerveControllerCommand  swerveCommand = new PPSwerveControllerCommand(
        examplePath, 
        drivetrain::getPose, 
        Constants.kinematics, 
        new PIDController(Constants.kPXController, 0, 0), 
        new PIDController(Constants.kPYController, 0, 0), 
        new PIDController(Constants.kPThetaController, 0, Constants.kDThetaController),  
        drivetrain::setModuleStates, 
        drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    //drivetrain.resetOdometry(examplePath.getInitialPose());

    // Run path following command, then stop at the end.
    return auto
    .andThen(() -> drivetrain.stopDrive());

  }

  private static double deadband(double value, double deadband) { 
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.2);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
