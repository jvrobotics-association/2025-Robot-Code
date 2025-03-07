package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeManiplulatorConstants;
import frc.robot.subsystems.AlgaeManipulator;

public class IntakeAlgae extends Command {
  public final AlgaeManipulator m_algaeManipulator;

  public IntakeAlgae(AlgaeManipulator algaeManipulator) {
    m_algaeManipulator = algaeManipulator;
    addRequirements(m_algaeManipulator);
  }

  @Override
  public void initialize() {
    m_algaeManipulator.setGrabberSpeed(AlgaeManiplulatorConstants.GRABBER_INTAKE_SPEED);
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(m_algaeManipulator.getGrabberVelocity()) < 5
        || m_algaeManipulator.getGrabberCurrent() > 8);
  }
}
