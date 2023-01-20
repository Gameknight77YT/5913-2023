// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class arm extends SubsystemBase {
  private TalonFX armMotor = new TalonFX(Constants.armMotorID);
  private TalonSRX linearActuator = new TalonSRX(Constants.linearActuatorID);
  private AnalogPotentiometer potentiometer = new AnalogPotentiometer(Constants.potentiometerID, 360*10);
  private PIDController armController = new PIDController(0, 0, 0);
  private PIDController actuatorController = new PIDController(0, 0, 0);

  /** Creates a new arm. */
  public arm() {
    armMotor.configFactoryDefault();
    linearActuator.configFactoryDefault();

    armMotor.setNeutralMode(NeutralMode.Brake);
    linearActuator.setNeutralMode(NeutralMode.Brake);

    armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    armMotor.setSelectedSensorPosition(0);
  }
  /**
   * 
   * @param preset 
   * 1 = starting config 
   * 2 = transition
   * 3 = middle
   * 4 = high
   */
  public void goToSetpoint(int preset){
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
