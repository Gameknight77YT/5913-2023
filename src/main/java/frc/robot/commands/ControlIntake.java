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
  private BooleanSupplier intakeCone;
  private BooleanSupplier outtakeCone;
  /** Creates a new ControlIntake. */
  public ControlIntake(Intake intake, BooleanSupplier intakeCone, BooleanSupplier outtakeCone) {
    this.intake = intake;
    this.intakeCone = intakeCone;
    this.outtakeCone = outtakeCone;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intakeCone.getAsBoolean()){
      intake.controlIntake(Constants.intakeSpeed);
    }else if(outtakeCone.getAsBoolean()){
      intake.controlIntake(-Constants.intakeSpeed);
    }else if((intakeCone.getAsBoolean() && outtakeCone.getAsBoolean()) == false){
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
