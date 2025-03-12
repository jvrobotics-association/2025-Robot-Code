package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class RetractClimber extends Command {
  private final Climber m_climber;

  public RetractClimber(Climber climber) {
    m_climber = climber;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {
    m_climber.releaseRatchet();
    m_climber.releaseChute();
    m_climber.stopRotationMotor();
  }

  @Override
  public void execute() {
    m_climber.retractClimber(ClimberConstants.RETRACT_SPEED);
  }

  @Override
  public boolean isFinished() {
    return m_climber.getClimberPidPosition() <= ClimberConstants.ROTATION_MIN;
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.stopWinchMotor();
    m_climber.engageRatchet();
  }
}
