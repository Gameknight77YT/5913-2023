// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ControlArm;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.HoldIntake;
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
  private final ControlArm controlArm;

  private final XboxController controllerDriver = new XboxController(0);
  private final XboxController controllerManipulator = new XboxController(1);

  private SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();

  private PathPlannerTrajectory Auto3High;
  private PathPlannerTrajectory Auto2High;
  private PathPlannerTrajectory Test;
  private PathPlannerTrajectory LowGoal;

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

    

    controlArm = new ControlArm(
      arm, 
      controllerManipulator::getLeftY, 
      controllerManipulator::getRightY,
      controllerManipulator::getBackButtonPressed,
      controllerManipulator::getStartButtonPressed
      );
    
    arm.setDefaultCommand(controlArm);

    InitTrajectorys();

    autoChooser.setDefaultOption("Auto2High", Auto2High);
    autoChooser.addOption("Auto3High", Auto3High);
    autoChooser.addOption("Test", Test);
    autoChooser.addOption("lowGoal", Auto2High);
    SmartDashboard.putData(autoChooser);
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
      .and(new Trigger(getLeftTrigger(controllerDriver)).negate())
      .whileTrue(new IntakeGamepiece(intake, true));

    new Trigger(getRightTrigger(controllerDriver))
      .and(new Trigger(getLeftTrigger(controllerDriver)))
      .whileTrue(new IntakeGamepiece(intake, false));

    new Trigger(controllerDriver::getRightBumper)
      .and(new Trigger(getLeftTrigger(controllerDriver)).negate())
      .whileTrue(new OuttakeGamepiece(intake, true));

    new Trigger(controllerDriver::getRightBumper)
      .and(new Trigger(getLeftTrigger(controllerDriver)))
      .whileTrue(new OuttakeGamepiece(intake, false));

    new Trigger(controllerDriver::getRightBumper)
      .and(getRightTrigger(controllerDriver))
      .whileFalse(new HoldIntake(intake));

    new Trigger(getRightTrigger(controllerManipulator))
      .onTrue(new MoveArmToSetpoint(arm, State.GroundPickup));

    new Trigger(controllerManipulator::getRightBumper)
      .onTrue(new MoveArmToSetpoint(arm, State.LoadingStation));

    new Trigger(controllerManipulator::getYButton)
      .and(new Trigger(controllerManipulator::getLeftBumper).negate())
      .onTrue(new MoveArmToSetpoint(arm, State.HighCone));
    
    new Trigger(controllerManipulator::getYButton)
      .and(controllerManipulator::getLeftBumper)
      .onTrue(new MoveArmToSetpoint(arm, State.HighCube));

    new Trigger(controllerManipulator::getXButton)
      .and(new Trigger(controllerManipulator::getLeftBumper).negate())
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

  public BooleanSupplier getLeftTrigger(XboxController controller){
    
    return new BooleanSupplier() {

      @Override
      public boolean getAsBoolean() {
        return controller.getLeftTriggerAxis() > .1;
      }
    };
  }

  private void InitTrajectorys() {
    Auto3High = PathPlanner.loadPath(
      "Auto3High",
      Constants.MAX_VELOCITY_METERS_PER_SECOND-1.5,
      Constants.MAX_acceleration_METERS_PER_SECOND-1.5);

    Auto2High = PathPlanner.loadPath(
      "Auto2High",
      Constants.MAX_VELOCITY_METERS_PER_SECOND-1.5,
      Constants.MAX_acceleration_METERS_PER_SECOND-1.5);

    Test = PathPlanner.loadPath(
      "Test",
      Constants.MAX_VELOCITY_METERS_PER_SECOND-1.5,
      Constants.MAX_acceleration_METERS_PER_SECOND-1.5);

    LowGoal = PathPlanner.loadPath(
      "LowGoal",
      Constants.MAX_VELOCITY_METERS_PER_SECOND-1.5,
      Constants.MAX_acceleration_METERS_PER_SECOND-1.5);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // This will load the file "Example Path.path" and generate it with a max velocity of 8 m/s and a max acceleration of 5 m/s^2
    

      HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("ToHighCone", new MoveArmToSetpoint(arm, State.HighCone).withTimeout(2.5));
      eventMap.put("ToHighCube", new MoveArmToSetpoint(arm, State.HighCube).withTimeout(2.5));
      eventMap.put("ToLow", new MoveArmToSetpoint(arm, State.NormalPickup));
      eventMap.put("ToStart", new MoveArmToSetpoint(arm, State.Starting).withTimeout(2.5));
      eventMap.put("IntakeCone", new IntakeGamepiece(intake, true).withTimeout(3));
      eventMap.put("IntakeCube", new IntakeGamepiece(intake, false).withTimeout(2));
      eventMap.put("OuttakeCone", new OuttakeGamepiece(intake, true).withTimeout(.5));
      eventMap.put("OuttakeCube", new OuttakeGamepiece(intake, false).withTimeout(1));
      //eventMap.put("AutoBalance", new AutoBalance(drivetrain));

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

        Command auto = autoBuilder.fullAuto(autoChooser.getSelected());
      
      
      

    // Reset odometry to the starting pose of the trajectory.
    //drivetrain.resetOdometry(examplePath.getInitialPose());

    // Run path following command, then stop at the end.
    return (new MoveArmToSetpoint(arm, State.Starting).withTimeout(.01))
    .andThen(auto)
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
