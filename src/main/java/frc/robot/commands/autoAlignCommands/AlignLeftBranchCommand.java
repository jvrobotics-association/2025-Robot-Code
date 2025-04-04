package frc.robot.commands.autoAlignCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AlignLeftBranchCommand extends Command {
  private final Drive m_drive;
  private Command pathCommand;
  private Pose2d targetPose;

  public AlignLeftBranchCommand(Drive drive) {
    m_drive = drive;
  }

  @Override
  public void initialize() {
    targetPose = FieldConstants.getNearestLeftBranch(m_drive.getPose());
    Logger.recordOutput("Tags/Chosen Left Branch", targetPose);

    pathCommand =
        AutoBuilder.followPath(
            new PathPlannerPath(
                PathPlannerPath.waypointsFromPoses(m_drive.getPose(), targetPose),
                AutoAlignConstants.PATH_CONSTRAINTS,
                null,
                new GoalEndState(0, targetPose.getRotation())) {
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
    return targetPose.getTranslation().getDistance(m_drive.getPose().getTranslation()) <= 0.01
        && Math.abs(targetPose.getRotation().minus(m_drive.getRotation()).getDegrees()) <= 0.75;
  }
}
