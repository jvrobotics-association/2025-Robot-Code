package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class RetractClimberCommand extends Command {
  private final ClimberSubsystem m_subsystem;

  public RetractClimberCommand(ClimberSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.retract();
  }

  @Override
  public boolean isFinished() {
    return m_subsystem.isRetracted();
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopClimber();
  }
}
