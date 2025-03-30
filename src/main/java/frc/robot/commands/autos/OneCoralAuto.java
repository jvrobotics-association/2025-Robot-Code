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
  private final String firstFacePathName;
  private final ReefAlignLocation firstScoreLocation;
  private final ElevatorHeight firstScoreHeight;

  public OneCoralAuto(
      Elevator elevator,
      CoralManipulator coralManipulator,
      String firstFacePathName,
      ReefAlignLocation firstScoreLocation,
      ElevatorHeight firstScoreHeight) {
    this.elevator = elevator;
    this.coralManipulator = coralManipulator;
    this.firstFacePathName = firstFacePathName;
    this.firstScoreLocation = firstScoreLocation;
    this.firstScoreHeight = firstScoreHeight;
  }

  @Override
  public void initialize() {
    try {
      PathPlannerPath firstFacePath = PathPlannerPath.fromPathFile(firstFacePathName);

      Command firstFacePathCommand = AutoBuilder.followPath(firstFacePath);

      Pose2d firstFacePathEndPoint =
          firstFacePath.getPathPoses().get(firstFacePath.getPathPoses().size() - 1);

      Pose2d firstScoreTarget;
      if (firstScoreLocation == ReefAlignLocation.LEFT) {
        firstScoreTarget = FieldConstants.getNearestLeftBranch(firstFacePathEndPoint);
      } else if (firstScoreLocation == ReefAlignLocation.RIGHT) {
        firstScoreTarget = FieldConstants.getNearestRightBranch(firstFacePathEndPoint);
      } else {
        firstScoreTarget = FieldConstants.getNearestReefFace(firstFacePathEndPoint);
      }

      Command firstFaceAlignCommand =
          AutoBuilder.followPath(
              new PathPlannerPath(
                  PathPlannerPath.waypointsFromPoses(firstFacePathEndPoint, firstScoreTarget),
                  AutoAlignConstants.PATH_CONSTRAINTS,
                  null,
                  new GoalEndState(0, firstScoreTarget.getRotation())) {
                {
                  preventFlipping = true;
                }
              });

      Commands.sequence(
              firstFacePathCommand,
              firstFaceAlignCommand,
              GamePieceCommands.placeCoralCommand(elevator, coralManipulator, firstScoreHeight))
          .schedule();

    } catch (Exception e) {
      System.out.println("Unable to run auto");
      e.printStackTrace();
    }
  }
}
