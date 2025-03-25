package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends Command {
  private final Elevator m_elevator;
  private final double m_height;
  private boolean hasCalledMove = false;

  public MoveElevator(Elevator elevator, ElevatorHeight height) {
    m_elevator = elevator;
    m_height = height.height;
    addRequirements(m_elevator);
  }

  public MoveElevator(Elevator elevator, double height) {
    m_elevator = elevator;
    m_height = height;
    addRequirements(m_elevator);
  }

  @Override
  public void execute() {
    if (!RobotContainer.isCoralDetected() && !hasCalledMove) {
      m_elevator.moveToPosition(m_height);
      hasCalledMove = true;
    }
  }

  @Override
  public boolean isFinished() {
    if (!RobotContainer.isCoralDetected()) {
      return Math.abs(m_elevator.getPidPosition() - m_elevator.getPosition()) <= 0.015;
    } else {
      return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    hasCalledMove = false;

    if (m_height == ElevatorHeight.L1.height) {
      m_elevator.stopElevator();
    }
  }
}
