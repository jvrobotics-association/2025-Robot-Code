package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

class ExtendClimber extends Command {
  private final Climber m_climber;

  ExtendClimber(Climber climber) {
    m_climber = climber;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {
    m_climber.extendClimber();
    m_climber.extendWinch();
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_climber.getClimberPidPosition() - m_climber.getClimberPosition()) <= 0.05;
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.stopWinchMotor();
  }
}
