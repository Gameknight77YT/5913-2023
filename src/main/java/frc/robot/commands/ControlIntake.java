// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ControlIntake extends CommandBase {
  private Intake intake;
  private BooleanSupplier intakeSupplier, outtakeSupplier, isConeSupplier;
  private Boolean isIntake, isOuttake, isCone;
  /** Creates a new ControlIntake. */
  public ControlIntake(Intake intake, BooleanSupplier intakeSupplier, BooleanSupplier outtakeSupplier, BooleanSupplier isConeSupplier) {
    this.intake = intake;
    this.intakeSupplier = intakeSupplier;
    this.outtakeSupplier = outtakeSupplier;
    this.isConeSupplier = isConeSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    isIntake = intakeSupplier.getAsBoolean();
    isOuttake = outtakeSupplier.getAsBoolean();
    isCone = isConeSupplier.getAsBoolean();

    
    intake.setLEDsCone(isCone);
    
    

    if(isIntake){
      if(!isCone){
        intake.controlIntake(Constants.intakeSpeed);
      }else{
        intake.controlIntake(-Constants.intakeSpeed);
      }
    }else if(isOuttake){
      if(!isCone){
        intake.controlIntake(-Constants.intakeSpeed);
      }else{
        intake.controlIntake(Constants.intakeSpeed-.2);
      }
    }else{
      intake.controlIntake(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.controlIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
