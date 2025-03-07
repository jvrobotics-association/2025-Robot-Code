package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends Command {
  private final Elevator m_elevator;
  private final ElevatorHeight m_Height;

  public MoveElevator(Elevator elevator, ElevatorHeight height) {
    m_elevator = elevator;
    m_Height = height;
    addRequirements(m_elevator);
  }

  @Override
  public void initialize() {
    m_elevator.moveToPosition(m_Height.height);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_elevator.getPosition() - m_elevator.getPidPosition()) <= 0.01;
  }
}
