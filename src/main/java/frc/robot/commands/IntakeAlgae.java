package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeManiplulatorConstants;
import frc.robot.subsystems.AlgaeManipulator;

public class IntakeAlgae extends Command {
  public final AlgaeManipulator m_algaeManipulator;
  private boolean hasReachedSpeed = false;

  public IntakeAlgae(AlgaeManipulator algaeManipulator) {
    m_algaeManipulator = algaeManipulator;
    addRequirements(m_algaeManipulator);
  }

  @Override
  public void initialize() {
    m_algaeManipulator.setGrabberSpeed(AlgaeManiplulatorConstants.GRABBER_INTAKE_SPEED);
  }

  @Override
  public void execute() {
    if (Math.abs(m_algaeManipulator.getGrabberVelocity()) >= 500) {
      hasReachedSpeed = true;
    }
  }

  @Override
  public boolean isFinished() {
    return hasReachedSpeed && (Math.abs(m_algaeManipulator.getGrabberVelocity()) < 200);
  }

  @Override
  public void end(boolean interrupted) {
    // m_algaeManipulator.stopGrabber();
    hasReachedSpeed = false;
  }
}
