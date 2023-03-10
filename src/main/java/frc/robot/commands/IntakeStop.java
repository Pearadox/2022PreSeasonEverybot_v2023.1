// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeStop extends CommandBase {

  private Intake _intake;

  /** Creates a new IntakeStop. */
  public IntakeStop(Intake intake) {
    _intake = intake;
    addRequirements(_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // do ur mom
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _intake.stopIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
