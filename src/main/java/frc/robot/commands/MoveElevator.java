package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends Command {
  private final Elevator m_elevator;
  private final CoralManipulator m_coralManipulator;
  private final ElevatorHeight m_Height;

  public MoveElevator(Elevator elevator, CoralManipulator coralManipulator, ElevatorHeight height) {
    m_elevator = elevator;
    m_coralManipulator = coralManipulator;
    m_Height = height;
    addRequirements(m_elevator, m_coralManipulator);
  }

  @Override
  public void initialize() {
    if (!m_coralManipulator.getCoralSensorDetected()) {
      m_elevator.moveToPosition(m_Height.height);
    }
  }

  @Override
  public boolean isFinished() {
    if (m_coralManipulator.getCoralSensorDetected()) {
      return true;
    } else {
      return Math.abs(m_elevator.getPosition() - m_elevator.getPidPosition()) <= 0.01;
    }
  }
}
