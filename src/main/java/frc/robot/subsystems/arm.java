// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private TalonSRX armMotor = new TalonSRX(Constants.armMotorID);
  private TalonSRX linearActuatorMaster = new TalonSRX(Constants.linearActuatorMasterID);
  private VictorSPX linearActuatorFollower = new VictorSPX(Constants.linearActuatorSlaveID);
  private AnalogPotentiometer potentiometer = new AnalogPotentiometer(Constants.potentiometerID, 360*10);
  private PIDController armController = new PIDController(0, 0, 0);
  private PIDController linearActuatorController = new PIDController(0, 0, 0);

  public static enum State {
    Starting,
    NormalPickup,
    GroundPickup,
    MiddleCone,
    MiddleCube,
    HighCone,
    HighCube,
    LoadingStation
  }

  public State currentState = State.Starting;

  /** Creates a new Arm. */
  public Arm() {
    armMotor.configFactoryDefault();
    linearActuatorMaster.configFactoryDefault();
    linearActuatorFollower.follow(linearActuatorMaster);

    armMotor.setNeutralMode(NeutralMode.Brake);
    linearActuatorMaster.setNeutralMode(NeutralMode.Brake);
    linearActuatorFollower.setNeutralMode(NeutralMode.Brake);

    armMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
    armMotor.configFeedbackNotContinuous(true, 0);

    armMotor.setInverted(true);
    linearActuatorMaster.setInverted(true);
    linearActuatorFollower.setInverted(InvertType.FollowMaster);
  }

  public void controlArm(double armSpeed, double actuatorSpeed){
    armMotor.set(TalonSRXControlMode.PercentOutput, MathUtil.applyDeadband(armSpeed, .15));
    linearActuatorMaster.set(TalonSRXControlMode.PercentOutput, MathUtil.applyDeadband(actuatorSpeed, .15));
  }
  
  /**
   * 
   * @param preset 
   */
  public void setState(State preset){
    currentState = preset;
  }

  public boolean isAtSetpoint(){
    return armController.atSetpoint() && linearActuatorController.atSetpoint();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (currentState) {
      case Starting:
        linearActuatorController.setSetpoint(Constants.linearActuatorStartingSetpoint);
        armController.setSetpoint(Constants.ArmStartingSetpoint);
        break;

      case NormalPickup:
        linearActuatorController.setSetpoint(Constants.linearActuatortNormalPickupSetpoint);
        armController.setSetpoint(Constants.ArmNormalPickupSetpoint);
        break;

      case GroundPickup:
        linearActuatorController.setSetpoint(Constants.linearActuatortGroundPickupSetpoint);
        armController.setSetpoint(Constants.ArmNormalGroundPickupSetpoint);
        break;

      case MiddleCone:
        linearActuatorController.setSetpoint(Constants.linearActuatorMiddleConeSetpoint);
        armController.setSetpoint(Constants.ArmMiddleConeSetpoint);
        break;

      case MiddleCube:
        linearActuatorController.setSetpoint(Constants.linearActuatorMiddleCubeSetpoint);
        armController.setSetpoint(Constants.ArmMiddleCubeSetpoint);
        break;

      case HighCone:
        linearActuatorController.setSetpoint(Constants.linearActuatorHighConeSetpoint);
        armController.setSetpoint(Constants.ArmHighConeSetpoint);
        break;

      case HighCube:
        linearActuatorController.setSetpoint(Constants.linearActuatorHighCubeSetpoint);
        armController.setSetpoint(Constants.ArmHighCubeSetpoint);
        break;

      case LoadingStation:
        linearActuatorController.setSetpoint(Constants.linearActuatorLoadingStationSetpoint);
        armController.setSetpoint(Constants.ArmLoadingStationSetpoint);
        break;
    
      default:
      DriverStation.reportError("invalid value", null);
        break;
    }

    controlArm(
      armController.calculate(armMotor.getSelectedSensorPosition()), 
      linearActuatorController.calculate(potentiometer.get())
      );
  }
}
