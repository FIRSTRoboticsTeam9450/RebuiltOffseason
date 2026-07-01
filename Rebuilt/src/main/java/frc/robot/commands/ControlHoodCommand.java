// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.AngleState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

/* Default command to control hood angle (tuning) */
public class ControlHoodCommand extends Command {
  private ShooterSubsystem shooter;
  private CommandXboxController control;

  private boolean prevDpadUp = false;
  private boolean prevDpadDown = false;

  /** Creates a new ControlHoodCommand. */
  public ControlHoodCommand(ShooterSubsystem shooterSub, CommandXboxController controller) {
    addRequirements(shooterSub);
    shooter = shooterSub;
    control = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.getCurrentAngleState() == AngleState.TUNING) {
      if (control.y().getAsBoolean()) shooter.setTuningAngleSetpoint(shooter.hoodAngleMax);
      else if (control.a().getAsBoolean()) shooter.setTuningAngleSetpoint(shooter.hoodAngleMin);
      else {
        double angleMax = shooter.hoodAngleMax - shooter.hoodAngleMin;
        double anglemin = shooter.hoodAngleMin;
        shooter.setTuningAngleSetpoint((control.getLeftY() * angleMax) + anglemin);
      }

      if (control.b().getAsBoolean()) shooter.setState(ShooterState.SHOOTING, AngleState.TUNING);
      else if (control.x().getAsBoolean()) shooter.setState(ShooterState.NOT_SHOOTING, AngleState.TUNING);

      boolean dpadUp = control.povUp().getAsBoolean();
      boolean dpadDown = control.povDown().getAsBoolean();
      if (dpadUp && !prevDpadUp) shooter.addManualVelocityShift();
      if (dpadDown && !prevDpadDown) shooter.subtractManualVelocityShift();
      prevDpadUp = dpadUp;
      prevDpadDown = dpadDown;

      if (control.rightBumper().getAsBoolean()) shooter.setState(ShooterState.NOT_SHOOTING, AngleState.STORING);
    } else if (control.leftBumper().getAsBoolean()) shooter.setState(ShooterState.NOT_SHOOTING, AngleState.TUNING);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
