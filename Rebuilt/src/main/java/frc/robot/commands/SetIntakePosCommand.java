// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePos;

/* Sets the intake position without setting the state of the wheels */
public class SetIntakePosCommand extends Command {
  private IntakeSubsystem intake;
  private IntakePos pos;

  /** Creates a new SetIntakePosCommand. */
  public SetIntakePosCommand(IntakeSubsystem intakeSub, IntakePos intakePos) {
    addRequirements(intakeSub);
    intake = intakeSub;
    pos = intakePos;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setPos(pos);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
