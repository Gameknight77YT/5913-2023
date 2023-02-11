// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.State;

public class MoveArmToSetpoint extends CommandBase {
  private Arm arm;
  private State state;
  private Boolean passedTansition = false;
  private Timer timer = new Timer();

  /** Creates a new MoveArmToSetpoint. 
   *  @param arm
   *  @param preset 
  */
  public MoveArmToSetpoint(Arm arm, State preset) {
    this.arm = arm;
    this.state = preset;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if((arm.currentState == State.Starting && (state == State.NormalPickup || state == State.GroundPickup)) || 
      (arm.currentState == State.NormalPickup && state == State.Starting)  || 
      (arm.currentState == State.GroundPickup && state == State.Starting)
      ){
        timer.reset();
        
      }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((arm.currentState == State.Starting && (state == State.NormalPickup || state == State.GroundPickup)) || 
      (arm.currentState == State.NormalPickup && state == State.Starting)  || 
      (arm.currentState == State.GroundPickup && state == State.Starting) ||
      (arm.currentState == State.Tansition)
      ){
        arm.setState(State.Tansition);
        if(arm.isAtSetpoint()){
          passedTansition = true;
        }
      }else{
        passedTansition = true;
      }
    
    if(passedTansition){
      arm.setState(state);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isAtSetpoint() && passedTansition;
  }
}
