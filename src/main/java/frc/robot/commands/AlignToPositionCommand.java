package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;

public class AlignToPositionCommand extends Command {
  private final Drive m_drive;
  private final Pose2d m_endPosition;
  private Command pathCommand;

  public AlignToPositionCommand(Drive drive, Pose2d endPosition) {
    m_drive = drive;
    m_endPosition = endPosition;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    pathCommand =
        AutoBuilder.followPath(
            new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(m_drive.getPose(), m_endPosition),
                new PathConstraints(
                    LinearVelocity.ofBaseUnits(0.9, MetersPerSecond),
                    LinearAcceleration.ofBaseUnits(1.5, MetersPerSecondPerSecond),
                    AngularVelocity.ofBaseUnits(540, DegreesPerSecond),
                    AngularAcceleration.ofBaseUnits(720, DegreesPerSecondPerSecond)),
                null,
                new GoalEndState(0, m_endPosition.getRotation())) {
              {
                preventFlipping = true;
              }
            });

    pathCommand.schedule();
  }

  @Override
  public void end(boolean interrupted) {
    Optional.ofNullable(pathCommand).ifPresent(Command::cancel);
  }

  @Override
  public boolean isFinished() {
    return m_endPosition.getTranslation().getDistance(m_drive.getPose().getTranslation()) <= 0.02
        && Math.abs(m_endPosition.getRotation().minus(m_drive.getRotation()).getDegrees()) <= 1.5;
  }
}
