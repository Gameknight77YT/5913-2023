// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.attribute.standard.MediaSize.Other;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private TalonFX armMotor = new TalonFX(Constants.armMotorID);
  private CANCoder armCanCoder = new CANCoder(Constants.ArmCANCoderID);
  private TalonSRX linearActuatorMaster = new TalonSRX(Constants.linearActuatorMasterID);
  private VictorSPX linearActuatorFollower = new VictorSPX(Constants.linearActuatorSlaveID);
  private AnalogPotentiometer potentiometer = new AnalogPotentiometer(Constants.potentiometerID, 360*10);
  public PIDController armController = new PIDController(.027, 0.001, 0.0001);
  public PIDController linearActuatorController = new PIDController(.003, 0, 0);

  private Compressor compressor = new Compressor(Constants.pcmID, PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid inArmPistons = new DoubleSolenoid(Constants.pcmID, PneumaticsModuleType.CTREPCM, Constants.inArmPistonsForward, Constants.inArmPistonsReverse);
  private DoubleSolenoid intakePistons = new DoubleSolenoid(Constants.pcmID, PneumaticsModuleType.CTREPCM, Constants.intakePistonsForward, Constants.intakePistonsReverse);

  public static enum State {
    Starting,
    NormalPickup,
    Tansition,
    GroundPickup,
    MiddleCone,
    MiddleCube,
    HighCone,
    HighCube,
    LoadingStation,
    other
  }

  public State currentState = State.other;
  public boolean armFirst;

  /** Creates a new Arm. */
  public Arm() {
    armMotor.configFactoryDefault();
    armCanCoder.configFactoryDefault();
    linearActuatorMaster.configFactoryDefault();
    linearActuatorFollower.follow(linearActuatorMaster);

    armMotor.setNeutralMode(NeutralMode.Brake);
    linearActuatorMaster.setNeutralMode(NeutralMode.Brake);
    linearActuatorFollower.setNeutralMode(NeutralMode.Brake);

    armMotor.setInverted(true);
    linearActuatorMaster.setInverted(true);
    linearActuatorFollower.setInverted(InvertType.FollowMaster);
    armCanCoder.configSensorDirection(true);

    armCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    armCanCoder.configMagnetOffset(360 - 260);
    
    compressor.enableDigital();

    inArmPistons.set(Value.kReverse);
    intakePistons.set(Value.kReverse);

    
    armController.setTolerance(1);
    linearActuatorController.setTolerance(1);
  }

  public void controlPistons(Value inArm, Value intake){
    inArmPistons.set(inArm);
    intakePistons.set(intake);
  }

  public void toggleInArmPistions(){
    inArmPistons.toggle();
  }

  public void toggleIntakePistions(){
    intakePistons.toggle();
  }

  public boolean armAtSetpoint(){
    return (Math.abs(armController.getSetpoint() - armCanCoder.getAbsolutePosition()) < 50);
  }

  public boolean actuatorAtSetpoint(){
    return (Math.abs(linearActuatorController.getSetpoint() - potentiometer.get()) < 1000);
  }

  public void controlArm(){
    double armSpeed = -armController.calculate(armCanCoder.getAbsolutePosition());
    double actuatorSpeed = -linearActuatorController.calculate(potentiometer.get());

    if(armFirst && !armAtSetpoint()){
      armMotor.set(TalonFXControlMode.PercentOutput, armSpeed);
      linearActuatorMaster.set(TalonSRXControlMode.PercentOutput, 0);
    }else if(armFirst && armAtSetpoint()){
      armMotor.set(TalonFXControlMode.PercentOutput, armSpeed);
      linearActuatorMaster.set(TalonSRXControlMode.PercentOutput, actuatorSpeed);
    }else if(!armFirst && !actuatorAtSetpoint()){
      armMotor.set(TalonFXControlMode.PercentOutput, 0);
      linearActuatorMaster.set(TalonSRXControlMode.PercentOutput, actuatorSpeed);
    }else if(!armFirst && actuatorAtSetpoint()){
      armMotor.set(TalonFXControlMode.PercentOutput, armSpeed);
      linearActuatorMaster.set(TalonSRXControlMode.PercentOutput, actuatorSpeed);
    }
    
  }

  public void controlArm(double armSpeed, double actuatorSpeed){
    armMotor.set(TalonFXControlMode.PercentOutput, armSpeed);
    linearActuatorMaster.set(TalonSRXControlMode.PercentOutput, actuatorSpeed);
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
    SmartDashboard.putString("state", currentState.toString());
    SmartDashboard.putNumber("armCancoder", armCanCoder.getAbsolutePosition());
    SmartDashboard.putNumber("potentiometer", potentiometer.get());
    //SmartDashboard.putData(armController);
    //SmartDashboard.putData(linearActuatorController);
    // This method will be called once per scheduler run
    switch (currentState) {
      case Starting:
        linearActuatorController.setSetpoint(Constants.linearActuatorStartingSetpoint);
        armController.setSetpoint(Constants.ArmStartingSetpoint);
        inArmPistons.set(Value.kReverse);
        intakePistons.set(Value.kReverse);
        break;

      case Tansition:
        linearActuatorController.setSetpoint(Constants.linearActuatorTansitionSetpoint);
        armController.setSetpoint(Constants.ArmTansitionSetpoint);
        inArmPistons.set(Value.kReverse);
        intakePistons.set(Value.kReverse);
        break;

      case NormalPickup:
        linearActuatorController.setSetpoint(Constants.linearActuatortNormalPickupSetpoint);
        armController.setSetpoint(Constants.ArmNormalPickupSetpoint);
        inArmPistons.set(Value.kReverse);
        intakePistons.set(Value.kForward);
        break;

      case GroundPickup:
        linearActuatorController.setSetpoint(Constants.linearActuatortGroundPickupSetpoint);
        armController.setSetpoint(Constants.ArmNormalGroundPickupSetpoint);
        inArmPistons.set(Value.kForward);
        intakePistons.set(Value.kForward);
        break;

      case MiddleCone:
        linearActuatorController.setSetpoint(Constants.linearActuatorMiddleConeSetpoint);
        armController.setSetpoint(Constants.ArmMiddleConeSetpoint);
        inArmPistons.set(Value.kReverse);
        intakePistons.set(Value.kForward);
        break;

      case MiddleCube:
        linearActuatorController.setSetpoint(Constants.linearActuatorMiddleCubeSetpoint);
        armController.setSetpoint(Constants.ArmMiddleCubeSetpoint);
        inArmPistons.set(Value.kReverse);
        intakePistons.set(Value.kForward);
        break;

      case HighCone:
        linearActuatorController.setSetpoint(Constants.linearActuatorHighConeSetpoint);
        armController.setSetpoint(Constants.ArmHighConeSetpoint);
        inArmPistons.set(Value.kReverse);
        intakePistons.set(Value.kForward);
        break;

      case HighCube:
        linearActuatorController.setSetpoint(Constants.linearActuatorHighCubeSetpoint);
        armController.setSetpoint(Constants.ArmHighCubeSetpoint);
        inArmPistons.set(Value.kReverse);
        intakePistons.set(Value.kForward);
        break;

      case LoadingStation:
        linearActuatorController.setSetpoint(Constants.linearActuatorLoadingStationSetpoint);
        armController.setSetpoint(Constants.ArmLoadingStationSetpoint);
        inArmPistons.set(Value.kReverse);
        intakePistons.set(Value.kForward);
        break;

      case other:
        
        break;

      default:
        DriverStation.reportError("invalid value", null);
        break;
    }
    if(!(currentState == State.other)){
      controlArm();
    }
    
  }
}
