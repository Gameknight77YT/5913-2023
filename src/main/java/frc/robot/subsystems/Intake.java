// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private TalonFX intakeMotor = new TalonFX(Constants.intakeMotorID);
  public PIDController pidController = new PIDController(.0005, 0, 0);
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.configFactoryDefault();

    intakeMotor.setNeutralMode(NeutralMode.Brake);
    
    intakeMotor.clearStickyFaults();

    intakeMotor.configOpenloopRamp(0);

    intakeMotor.setInverted(true);

    intakeMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    pidController.setTolerance(1000);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run\
    SmartDashboard.putNumber("encoder", intakeMotor.getSelectedSensorPosition());
  }

  public void controlIntake(double speed){
    if(speed != 0){
      intakeMotor.set(TalonFXControlMode.PercentOutput, speed);
    }else if(speed == 0){
      if(Math.abs(pidController.getSetpoint() - intakeMotor.getSelectedSensorPosition()) >= 1000){
        pidController.setSetpoint(intakeMotor.getSelectedSensorPosition());
      }
      intakeMotor.set(TalonFXControlMode.PercentOutput, pidController.calculate(intakeMotor.getSelectedSensorPosition())/5);
      
    }
  }
}
