// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.State;

public class ControlArm extends CommandBase {
  private Arm arm;
  private DoubleSupplier armSpeed;
  private DoubleSupplier actuatorSpeed;
  private BooleanSupplier inarmPistionToggle;
  private BooleanSupplier intakePistionToggle;
  /** Creates a new ControlArm. */
  public ControlArm(Arm arm, DoubleSupplier armSpeed, DoubleSupplier actuatorSpeed, BooleanSupplier inarmPistonToggle, BooleanSupplier intakePistionToggle) {
    this.arm = arm;
    this.armSpeed = armSpeed;
    this.actuatorSpeed = actuatorSpeed;
    this.inarmPistionToggle = inarmPistonToggle;
    this.intakePistionToggle = intakePistionToggle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(armSpeed.getAsDouble()) > .15 || Math.abs(actuatorSpeed.getAsDouble()) > .15){
      arm.currentState = State.other;
    }
    arm.controlArm(MathUtil.applyDeadband(armSpeed.getAsDouble(), .15), MathUtil.applyDeadband(actuatorSpeed.getAsDouble(), .15));
    if(inarmPistionToggle.getAsBoolean()) arm.toggleInArmPistions();
    if(intakePistionToggle.getAsBoolean()) arm.toggleIntakePistions();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.controlArm(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
