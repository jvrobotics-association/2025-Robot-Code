package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeManiplulatorConstants;
import frc.robot.subsystems.AlgaeGrabber;

public class IntakeAlgae extends Command {
  public final AlgaeGrabber m_AlgaeGrabber;
  private boolean hasReachedSpeed = false;

  public IntakeAlgae(AlgaeGrabber algaeGrabber) {
    m_AlgaeGrabber = algaeGrabber;
    addRequirements(m_AlgaeGrabber);
  }

  @Override
  public void initialize() {
    m_AlgaeGrabber.setGrabberSpeed(AlgaeManiplulatorConstants.GRABBER_INTAKE_SPEED);
  }

  @Override
  public void execute() {
    if (Math.abs(m_AlgaeGrabber.getGrabberVelocity()) >= 500) {
      hasReachedSpeed = true;
    }
  }

  @Override
  public boolean isFinished() {
    return hasReachedSpeed && (Math.abs(m_AlgaeGrabber.getGrabberVelocity()) < 200);
  }

  @Override
  public void end(boolean interrupted) {
    hasReachedSpeed = false;
  }
}
