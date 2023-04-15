// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.State;

public class Intake extends SubsystemBase {
  private TalonSRX intakeMotor = new TalonSRX(Constants.intakeMotorID);
  public PIDController pidController = new PIDController(.00050, 0, 0);
  private double lastStallTime = 0;
  private double startTime = 0;
  private boolean isStall = false;
  private boolean hasStalled = false;

  private CANdle candle = new CANdle(Constants.CANdleID); 
  private Arm arm;

  private boolean isCone = true;
  
  /** Creates a new Intake. */
  public Intake(Arm arm) {
    this.arm = arm;
    candle.configFactoryDefault();
    candle.configLOSBehavior(false);
    candle.configLEDType(LEDStripType.GRB);
    
    
    intakeMotor.configFactoryDefault();

    intakeMotor.setNeutralMode(NeutralMode.Brake);
    
    intakeMotor.clearStickyFaults();

    intakeMotor.configOpenloopRamp(0);

    intakeMotor.setInverted(false);

    intakeMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    //intakeMotor.setSelectedSensorPosition(0);

    pidController.setTolerance(1000);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("encoder", intakeMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("speed", intakeMotor.getSelectedSensorVelocity());
    //SmartDashboard.putNumber("volt", intakeMotor.getBusVoltage());
    //SmartDashboard.putNumber("voltOut", intakeMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("current", intakeMotor.getSupplyCurrent());
    SmartDashboard.putBoolean("isStall", isStall);
    //SmartDashboard.putNumber("time", RobotController.getFPGATime()/1000000);
    if(DriverStation.isDisabled() || DriverStation.isAutonomous()){
      if(DriverStation.getAlliance() == Alliance.Red){
        candle.setLEDs(255, 0, 0);
      }else if(DriverStation.getAlliance() == Alliance.Blue){
        candle.setLEDs(0, 0, 255);
      }
    }else if(arm.currentState != State.HighCone && arm.currentState != State.MiddleCone){
      if(isCone){
        candle.setLEDs(255, 0, 255);
      }else{
        if(DriverStation.getAlliance() == Alliance.Red){
          candle.setLEDs(255, 0, 0);
        }else if(DriverStation.getAlliance() == Alliance.Blue){
          candle.setLEDs(0, 0, 255);
        }
      }
    }else{
      candle.setLEDs(0, 0, 0);
    }
  }

  public void setLEDsCone(boolean isCone){
    this.isCone = isCone;
  }

  

  public void controlIntake(double speed){
    if(speed != 0){
      double time = RobotController.getFPGATime();
      
      if(intakeMotor.getSelectedSensorVelocity() == 0){
        startTime = time;
        
      }
      if(time - startTime > 2*1000000){
        if(intakeMotor.getSupplyCurrent() > 70 && intakeMotor.getSelectedSensorVelocity() > 0 ){
        
          hasStalled = true;
          lastStallTime = time;
          
        }else if(intakeMotor.getSupplyCurrent() > 70 && intakeMotor.getSelectedSensorVelocity() < 0 ){
          hasStalled = true;
          lastStallTime = time;
          
        }
        
      }
      if(hasStalled && time - lastStallTime > .5*1000000){
        if(2000 > intakeMotor.getSelectedSensorVelocity() && intakeMotor.getSelectedSensorVelocity() > 0 ){
          isStall = true;
          lastStallTime = time;
          
        }else if(-6000 > intakeMotor.getSelectedSensorVelocity() && intakeMotor.getSelectedSensorVelocity() < 0 ){
          isStall = true;
          lastStallTime = time;
          
        }else{
          hasStalled = false;
        }
      }
      
      if(isStall && hasStalled && time - lastStallTime < .5*1000000){
        speed = 0;
        
      }else if(isStall && hasStalled && time - lastStallTime > .5*1000000){
        isStall = false;
        hasStalled = false;
      }

      pidController.setSetpoint(intakeMotor.getSelectedSensorPosition());
      intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }else if(speed == 0){
      isStall = false;
      if(Math.abs(pidController.getSetpoint() - intakeMotor.getSelectedSensorPosition()) >= 500){
        pidController.setSetpoint(intakeMotor.getSelectedSensorPosition());
      }
      intakeMotor.set(TalonSRXControlMode.PercentOutput, pidController.calculate(intakeMotor.getSelectedSensorPosition()));
      
    }
  }
}
