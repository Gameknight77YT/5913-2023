// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ControlArm;
import frc.robot.commands.ControlIntake;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.GetCube;
import frc.robot.commands.IntakeGamepiece;
import frc.robot.commands.MoveArmToSetpoint;
import frc.robot.commands.OuttakeGamepiece;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.State;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Camera camera;
  private final Drivetrain drivetrain;
  private final Intake intake;
  private final Arm arm;
  

  private final DefaultDriveCommand defaultDriveCommand;
  private final ControlArm controlArm;
  private final ControlIntake controlIntake;

  private final XboxController controllerDriver = new XboxController(0);
  private final XboxController controllerManipulator = new XboxController(1);

  private SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();

  
  private PathPlannerTrajectory Auto2HighNoCharge;
  private PathPlannerTrajectory Auto3HighNoCharge;
  private PathPlannerTrajectory Auto2High;
  private PathPlannerTrajectory Test;
  private PathPlannerTrajectory OneCone;
  private PathPlannerTrajectory OneConeNoCharge;
  private PathPlannerTrajectory SideAuto;
  private PathPlannerTrajectory Auto1Mid2High;
  private PathPlannerTrajectory Smooth3_1;
  private PathPlannerTrajectory Smooth3_2;
  private PathPlannerTrajectory Smooth3_3;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain = new Drivetrain();
    arm = new Arm();
    intake = new Intake(arm);
    camera = new Camera(arm);
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    defaultDriveCommand = new DefaultDriveCommand(
      drivetrain, camera,
      () -> -modifyAxis(controllerDriver.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controllerDriver.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controllerDriver.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
      controllerDriver::getAButton,
      controllerDriver::getXButton
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

    controlIntake = new ControlIntake(intake, 
      getRightTrigger(controllerDriver), 
      controllerDriver::getRightBumper, 
      getLeftTrigger(controllerDriver));

    intake.setDefaultCommand(controlIntake);

    InitTrajectorys();

    autoChooser.addOption("Auto2High", Auto2High);
    autoChooser.addOption("Auto2HighNoCharge", Auto2HighNoCharge);
    autoChooser.addOption("2.5 Smooth", Auto3HighNoCharge);
    autoChooser.addOption("Test", Test);
    autoChooser.addOption("OneCone", OneCone);
    autoChooser.addOption("OneConeNoCharge", OneConeNoCharge);
    autoChooser.setDefaultOption("3 Smooth", Auto1Mid2High);
    autoChooser.addOption("2.5 Bumb", SideAuto);
    autoChooser.addOption("3 Smooth Cube Tracking", Smooth3_1);
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

    new Trigger(getRightTrigger(controllerManipulator))
      .onTrue(new MoveArmToSetpoint(arm, State.GroundPickup));

    new Trigger(controllerManipulator::getRightBumper)
      .onTrue(new MoveArmToSetpoint(arm, State.LoadingStation));

    new Trigger(controllerManipulator::getRightBumper)
      .and(controllerManipulator::getLeftBumper)
      .onTrue(new MoveArmToSetpoint(arm, State.LoadingStationRamp));

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
    Auto2HighNoCharge = PathPlanner.loadPath(
      "Auto2HighNoCharge",
      Constants.MAX_VELOCITY_METERS_PER_SECOND-1,
      Constants.MAX_acceleration_METERS_PER_SECOND-1);

    Auto3HighNoCharge = PathPlanner.loadPath(
      "Auto3HighNoCharge",
      Constants.MAX_VELOCITY_METERS_PER_SECOND-.6,
      Constants.MAX_acceleration_METERS_PER_SECOND-.7);

    Auto2High = PathPlanner.loadPath(
      "Auto2High",
      Constants.MAX_VELOCITY_METERS_PER_SECOND-1,
      Constants.MAX_acceleration_METERS_PER_SECOND-1);

    Test = PathPlanner.loadPath(
      "Test",
      Constants.MAX_VELOCITY_METERS_PER_SECOND-1,
      Constants.MAX_acceleration_METERS_PER_SECOND-1);

    OneCone = PathPlanner.loadPath(
      "OneCone",
      Constants.MAX_VELOCITY_METERS_PER_SECOND-1,
      Constants.MAX_acceleration_METERS_PER_SECOND-1);
      
    OneConeNoCharge = PathPlanner.loadPath(
      "OneConeNoCharge",
      Constants.MAX_VELOCITY_METERS_PER_SECOND-1,
      Constants.MAX_acceleration_METERS_PER_SECOND-1);

    Auto1Mid2High = PathPlanner.loadPath(
      "Auto1Mid2High",
      Constants.MAX_VELOCITY_METERS_PER_SECOND,
      Constants.MAX_acceleration_METERS_PER_SECOND-.6);

    SideAuto = PathPlanner.loadPath(
        "SideAuto",
        Constants.MAX_VELOCITY_METERS_PER_SECOND-2.5,
        Constants.MAX_acceleration_METERS_PER_SECOND-.6);

    Smooth3_1 = PathPlanner.loadPath(
        "Smooth3_1",
        Constants.MAX_VELOCITY_METERS_PER_SECOND,
        Constants.MAX_acceleration_METERS_PER_SECOND);

    Smooth3_2 = PathPlanner.loadPath(
        "Smooth3_2",
        Constants.MAX_VELOCITY_METERS_PER_SECOND,
        Constants.MAX_acceleration_METERS_PER_SECOND);

    Smooth3_3 = PathPlanner.loadPath(
        "Smooth3_3",
        Constants.MAX_VELOCITY_METERS_PER_SECOND,
        Constants.MAX_acceleration_METERS_PER_SECOND);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // This will load the file "Example Path.path" and generate it with a max velocity of 8 m/s and a max acceleration of 5 m/s^2
    

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("ToHighCone", new MoveArmToSetpoint(arm, State.HighCone).withTimeout(2));
    eventMap.put("ToHighCube", new MoveArmToSetpoint(arm, State.HighCube).withTimeout(2.5));
    eventMap.put("ToMidCube", new MoveArmToSetpoint(arm, State.MiddleCube).withTimeout(1));
    eventMap.put("ToMidCone", new MoveArmToSetpoint(arm, State.MiddleCone).withTimeout(.5));
    eventMap.put("ToLow", new MoveArmToSetpoint(arm, State.NormalPickup).withTimeout(3.5));
    eventMap.put("ToStart", new MoveArmToSetpoint(arm, State.Starting).withTimeout(2));
    eventMap.put("ToStartShort", new MoveArmToSetpoint(arm, State.Starting).withTimeout(.5));
    eventMap.put("IntakeCone", new IntakeGamepiece(intake, true).withTimeout(3));
    eventMap.put("IntakeCube", new IntakeGamepiece(intake, false).withTimeout(1.25));
    eventMap.put("OuttakeCone", new OuttakeGamepiece(intake, true).withTimeout(.5));
    eventMap.put("OuttakeCube", new OuttakeGamepiece(intake, false).withTimeout(.5));
    eventMap.put("GetCube", new GetCube(drivetrain, camera).withTimeout(1));
    eventMap.put("AutoBalance", new DefaultDriveCommand(drivetrain, camera, () -> 0, () -> 0, 
                                                        () -> 0, () -> false, () -> true).withTimeout(1));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drivetrain::getPose, 
      drivetrain::resetOdometry, 
      Constants.kinematics, 
      new PIDConstants(Constants.kPXYController, 0, 0), 
      new PIDConstants(Constants.kPThetaController, 0, Constants.kDThetaController), 
      drivetrain::setModuleStates, 
      eventMap, 
      true,
      drivetrain
      );
      
    Command auto;
    if(autoChooser.getSelected() == Smooth3_1){
      auto = autoBuilder.fullAuto(Smooth3_1).withTimeout(3)
      .andThen(new GetCube(drivetrain, camera).withTimeout(.7)
        .alongWith(new IntakeGamepiece(intake, false)).withTimeout(1))
      .andThen(autoBuilder.followPathWithEvents(Smooth3_2).withTimeout(5.65))
      .andThen(new GetCube(drivetrain, camera).withTimeout(.6)
        .alongWith(new IntakeGamepiece(intake, false)).withTimeout(.75))
      .andThen(autoBuilder.followPathWithEvents(Smooth3_3))
        ;
    }else{
      auto = autoBuilder.fullAuto(autoChooser.getSelected());
    }  
      
      

    // Reset odometry to the starting pose of the trajectory.
    //drivetrain.resetOdometry(examplePath.getInitialPose());

    // Run path following command, then stop at the end.
    return (new MoveArmToSetpoint(arm, State.Starting).withTimeout(.001))
    //.alongWith(new InstantCommand(() -> drivetrain.zeroGyroscope()))
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
