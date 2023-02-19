// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.State;

public class MoveArmToSetpoint extends CommandBase {
  private Arm arm;
  private State state;
  private Boolean armFirst = null;
  private State currentState;
  private boolean passedTransition = false;
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

  public MoveArmToSetpoint(Arm arm, State preset, Boolean armFirst){
    this.armFirst = armFirst;
    new MoveArmToSetpoint(arm, preset);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(armFirst == null){
      if(currentState == State.Starting || (state == State.NormalPickup && currentState != State.Starting) ){
        armFirst = true;
      }else if(state == State.Starting || currentState == State.GroundPickup || 
        currentState == State.NormalPickup || currentState == State.HighCone || 
        currentState == State.HighCube || currentState == State.LoadingStation ||
        currentState == State.MiddleCone || currentState == State.MiddleCube || 
        state == State.NormalPickup){

        armFirst = false;
      }else{
        armFirst = true;
      }
    }
    arm.armFirst = armFirst;

    if((arm.currentState == State.Starting && (state == State.NormalPickup || state == State.GroundPickup)) || 
      (arm.currentState == State.NormalPickup && state == State.Starting)  || 
      (arm.currentState == State.GroundPickup && state == State.Starting)
      ){
        timer.reset();
        timer.start();
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
        if(arm.isAtSetpoint() || timer.get() > .75){
          passedTransition = true;
        }
      }else{
        passedTransition = true;
      }
    
    if(passedTransition){
      arm.setState(state);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    passedTransition = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isAtSetpoint() && passedTransition;
  }
}
