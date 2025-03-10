package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeManipulator;

public class MoveAlgaeArm extends Command {
  private final AlgaeManipulator m_AlgaeManipulator;
  private final double m_position;

  public MoveAlgaeArm(AlgaeManipulator algaeManipulator, double position) {
    m_AlgaeManipulator = algaeManipulator;
    m_position = position;
    addRequirements(m_AlgaeManipulator);
  }

  @Override
  public void initialize() {
    m_AlgaeManipulator.setRotationPosition(m_position);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_position - m_AlgaeManipulator.getPosition()) <= 0.2;
  }
}
