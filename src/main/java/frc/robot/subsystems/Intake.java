// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private TalonSRX intakeMotor = new TalonSRX(Constants.intakeMotorID);
  public PIDController pidController = new PIDController(.0009, 0, 0);
  private double lastStallTime = 0;
  private double startTime = 0;
  private boolean isStall = false;
  
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.configFactoryDefault();

    intakeMotor.setNeutralMode(NeutralMode.Brake);
    
    intakeMotor.clearStickyFaults();

    intakeMotor.configOpenloopRamp(0);

    intakeMotor.setInverted(false);

    intakeMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    intakeMotor.setSelectedSensorPosition(0);

    pidController.setTolerance(1000);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run\
    //SmartDashboard.putNumber("encoder", intakeMotor.getSelectedSensorPosition());
    //SmartDashboard.putNumber("speed", intakeMotor.getSelectedSensorVelocity());
    //SmartDashboard.putBoolean("isStall", isStall);
    //SmartDashboard.putNumber("time", RobotController.getFPGATime()/1000000);

  }

  public void controlIntake(double speed){
    if(speed != 0){
      double time = RobotController.getFPGATime();
      
      if(Math.abs(intakeMotor.getSelectedSensorVelocity()) == 100){
        startTime = time;
        
      }
      if(time - startTime > 1*1000000){
        if(2000 > intakeMotor.getSelectedSensorVelocity() && intakeMotor.getSelectedSensorVelocity() > 0 ){
          speed = 0;
          lastStallTime = time;
          isStall = true;
        }else if(-5000 > intakeMotor.getSelectedSensorVelocity() && intakeMotor.getSelectedSensorVelocity() < 0 ){
          speed = 0;
          lastStallTime = time;
          isStall = true;
        }
        
      }
      if(time - lastStallTime < 1*1000000){
        speed = 0;
        
      }
      
      intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }else if(speed == 0){
      isStall = false;
      if(Math.abs(pidController.getSetpoint() - intakeMotor.getSelectedSensorPosition()) >= 1000){
        pidController.setSetpoint(intakeMotor.getSelectedSensorPosition());
      }
      intakeMotor.set(TalonSRXControlMode.PercentOutput, pidController.calculate(intakeMotor.getSelectedSensorPosition())/2);
      
    }
  }
}
