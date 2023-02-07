// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.ControlArm;
import frc.robot.commands.ControlIntake;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IntakeGamepiece;
import frc.robot.commands.MoveArmToSetpoint;
import frc.robot.commands.OuttakeGamepiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm.State;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain;
  private final Intake intake;
  private final Arm arm;
  

  private final DefaultDriveCommand defaultDriveCommand;
  private final ControlIntake controlIntake;
  private final ControlArm controlArm;

  private final XboxController controllerDriver = new XboxController(0);
  private final XboxController controllerManipulator = new XboxController(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain = new Drivetrain();
    intake = new Intake();
    arm = new Arm();
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    defaultDriveCommand = new DefaultDriveCommand(
      drivetrain,
      () -> -modifyAxis(controllerDriver.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controllerDriver.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controllerDriver.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
      () -> controllerDriver.getAButton()//auto balance
    );
    
    drivetrain.setDefaultCommand(defaultDriveCommand);

    controlIntake = new ControlIntake(
      intake, 
      controllerManipulator::getAButton, 
      controllerManipulator::getBButton
      );


    intake.setDefaultCommand(controlIntake);

    controlArm = new ControlArm(
      arm, 
      controllerManipulator::getLeftY, 
      controllerManipulator::getRightY
      );
    
    arm.setDefaultCommand(controlArm);
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
    new Trigger(controllerDriver::getBackButton)
      .onTrue( new InstantCommand(() -> drivetrain.zeroGyroscope()));

    new Trigger(getRightTrigger(controllerDriver))
      .whileTrue(new IntakeGamepiece(intake, !controllerDriver.getLeftBumper()));

    new Trigger(controllerDriver::getRightBumper)
      .whileTrue(new OuttakeGamepiece(intake, !controllerDriver.getLeftBumper()));

    new Trigger(getRightTrigger(controllerManipulator))
      .onTrue(new MoveArmToSetpoint(arm, State.GroundPickup));

    new Trigger(controllerManipulator::getRightBumper)
      .onTrue(new MoveArmToSetpoint(arm, State.LoadingStation));

    new Trigger(controllerManipulator::getYButton)
      .and(controllerManipulator::getLeftBumper).negate()
      .onTrue(new MoveArmToSetpoint(arm, State.HighCone));
    
    new Trigger(controllerManipulator::getYButton)
      .and(controllerManipulator::getLeftBumper)
      .onTrue(new MoveArmToSetpoint(arm, State.HighCube));

    new Trigger(controllerManipulator::getXButton)
      .and(controllerManipulator::getLeftBumper).negate()
      .onTrue(new MoveArmToSetpoint(arm, State.MiddleCone));
    
    new Trigger(controllerManipulator::getXButton)
      .and(controllerManipulator::getLeftBumper)
      .onTrue(new MoveArmToSetpoint(arm, State.MiddleCube));
    
    new Trigger(controllerManipulator::getBButton)
      .onTrue(new MoveArmToSetpoint(arm, State.Starting));

    new Trigger(controllerManipulator::getAButton)
      .onTrue(new MoveArmToSetpoint(arm, State.NormalPickup));
    
    
  }

  public BooleanSupplier getRightTrigger(XboxController controller){
    
    return new BooleanSupplier() {

      @Override
      public boolean getAsBoolean() {
        return controller.getRightTriggerAxis() > .1;
      }
    };
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
      eventMap.put("StartToLow", new MoveArmToSetpoint(arm, State.GroundPickup));
      eventMap.put("LowToStart", new MoveArmToSetpoint(arm, State.Starting));
      eventMap.put("Intake", new IntakeGamepiece(intake, false));
      eventMap.put("Outtake", new OuttakeGamepiece(intake, false));
      eventMap.put("AutoBalance", new AutoBalance(drivetrain));

      SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        drivetrain::getPose, 
        drivetrain::resetOdometry, 
        Constants.kinematics, 
        new PIDConstants(Constants.kPXYController, 0, 0), 
        new PIDConstants(Constants.kPThetaController, 0, Constants.kDThetaController), 
        drivetrain::setModuleStates, 
        eventMap, 
        drivetrain
        );

        Command auto = autoBuilder.fullAuto(examplePath);

      
      

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
