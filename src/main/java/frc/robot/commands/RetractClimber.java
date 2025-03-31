package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

class RetractClimber extends Command {
  private final Climber m_climber;

  RetractClimber(Climber climber) {
    m_climber = climber;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {
    m_climber.stopRotationMotor();
  }

  @Override
  public void execute() {
    m_climber.retractClimber();
  }

  @Override
  public boolean isFinished() {
    return m_climber.getClimberPosition() <= ClimberConstants.ROTATION_MIN;
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.engageRatchet();
    m_climber.stopWinchMotor();
  }
}
