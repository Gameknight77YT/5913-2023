// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.State;

public class Camera extends SubsystemBase {
  private PhotonCamera mainCam = new PhotonCamera("OV5647");
  private PhotonCamera intakeCam = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  private PhotonPipelineResult mainCamResult = null;
  private PhotonPipelineResult intakeCamResult = null;

  private Arm arm;

  private PIDController pidController = new PIDController(0.05, 0, 0);

  /** Creates a new Camera. */
  public Camera(Arm arm) {
    this.arm = arm;
    pidController.setSetpoint(0);
    pidController.setTolerance(.3);
  }

  @Override
  public void periodic() {
    mainCamResult = mainCam.getLatestResult();
    intakeCamResult = intakeCam.getLatestResult();
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("moveinput", getMoveInput());

    if(arm.currentState == State.HighCone || arm.currentState == State.MiddleCone){
      mainCam.setLED(VisionLEDMode.kOn);
      intakeCam.setPipelineIndex(0);
    }else if(arm.currentState == State.NormalPickup){
      intakeCam.setPipelineIndex(1);
    }else{
      mainCam.setLED(VisionLEDMode.kOff);
    }
  }

  public double getMoveInput(){
    double input = 0;
    if(arm.currentState == State.HighCone || arm.currentState == State.MiddleCone){
      if(mainCamResult.hasTargets() && intakeCamResult.hasTargets()){
        input = pidController.calculate(mainCamResult.getBestTarget().getYaw() - intakeCamResult.getBestTarget().getYaw()/2.428);
      }else{
        input = 0;
      }
    }else if(arm.currentState == State.NormalPickup){
      if(intakeCamResult.hasTargets()){
        input = pidController.calculate(intakeCamResult.getBestTarget().getYaw()/2.428);
      }else{
        input = 0;
      }
    }
    if(Math.abs(input) < .1){
      input = 0;
    }


    return input;
  }
}
