package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefAlignLocation;
import frc.robot.commands.GamePieceCommands;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;

public class OneCoralAuto extends Command {
  private final Elevator elevator;
  private final CoralManipulator coralManipulator;
  private final ReefAlignLocation alignLocation;
  private final ElevatorHeight scoreHeight;
  private final String startPathName;

  public OneCoralAuto(
      Elevator elevator,
      CoralManipulator coralManipulator,
      ReefAlignLocation alignLocation,
      ElevatorHeight scoreHeight,
      String startPathName) {
    this.elevator = elevator;
    this.coralManipulator = coralManipulator;
    this.alignLocation = alignLocation;
    this.scoreHeight = scoreHeight;
    this.startPathName = startPathName;
  }

  @Override
  public void initialize() {
    try {
      PathPlannerPath startPath = PathPlannerPath.fromPathFile(startPathName);
      Pose2d startPathEndPoint = startPath.getPathPoses().get(startPath.getPathPoses().size() - 1);

      Pose2d targetPose;
      if (alignLocation == ReefAlignLocation.LEFT) {
        targetPose = FieldConstants.getNearestLeftBranch(startPathEndPoint);
      } else if (alignLocation == ReefAlignLocation.RIGHT) {
        targetPose = FieldConstants.getNearestRightBranch(startPathEndPoint);
      } else {
        targetPose = FieldConstants.getNearestReefFace(startPathEndPoint);
      }

      Command startToReefCommand = AutoBuilder.followPath(startPath);

      Command alignPathCommand =
          AutoBuilder.pathfindThenFollowPath(
              new PathPlannerPath(
                  PathPlannerPath.waypointsFromPoses(startPathEndPoint, targetPose),
                  AutoAlignConstants.PATH_CONSTRAINTS,
                  null,
                  new GoalEndState(0, targetPose.getRotation())) {
                {
                  preventFlipping = true;
                }
              },
              AutoAlignConstants.PATH_CONSTRAINTS);

      Commands.sequence(
              startToReefCommand,
              alignPathCommand,
              GamePieceCommands.placeCoralCommand(elevator, coralManipulator, scoreHeight))
          .schedule();

    } catch (Exception e) {
      System.out.println("Unable to run auto");
      e.printStackTrace();
    }
  }
}
