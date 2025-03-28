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

public class TwoCoralAuto extends Command {
  private final Elevator elevator;
  private final CoralManipulator coralManipulator;
  private final String startPathName;
  private final String reefToSourcePathName;
  private final String sourceToReefPathName;
  private final ReefAlignLocation firstScoreLocation;
  private final ElevatorHeight firstScoreHeight;
  private final ReefAlignLocation secondScoreLocation;
  private final ElevatorHeight secondScoreHeight;

  /**
   * Build the command for running a two coral auto
   *
   * @param elevator The elevator subsystem
   * @param coralManipulator The coral manipulator subsystem
   * @param startPathName The name of the path that will bring the robot from the starting position
   *     to the face of the reef for the first scoring location
   * @param reefToSourcePathName The name of the path that will bring the robot from the face of the
   *     first scoring location to the source
   * @param sourceToReefPathName The name of the path that will bring the robot from the source to
   *     the face of the reef for the second scoring location
   * @param firstScoreLocation The alignment position on the reef for the first scoring location
   * @param firstScoreHeight The height at wich to place the coral for the first scoring location
   * @param secondScoreLocation The alignment position on the reef for the secnd scoring location
   * @param secondScoreHeight The height at wich to place the coral for the second scoring location
   */
  public TwoCoralAuto(
      Elevator elevator,
      CoralManipulator coralManipulator,
      String startPathName,
      String reefToSourcePathName,
      String sourceToReefPathName,
      ReefAlignLocation firstScoreLocation,
      ElevatorHeight firstScoreHeight,
      ReefAlignLocation secondScoreLocation,
      ElevatorHeight secondScoreHeight) {
    this.elevator = elevator;
    this.coralManipulator = coralManipulator;
    this.startPathName = startPathName;
    this.reefToSourcePathName = reefToSourcePathName;
    this.sourceToReefPathName = sourceToReefPathName;
    this.firstScoreLocation = firstScoreLocation;
    this.firstScoreHeight = firstScoreHeight;
    this.secondScoreLocation = secondScoreLocation;
    this.secondScoreHeight = secondScoreHeight;
  }

  @Override
  public void initialize() {
    try {
      PathPlannerPath startPath = PathPlannerPath.fromPathFile(startPathName);
      PathPlannerPath reefToSourcePath = PathPlannerPath.fromPathFile(reefToSourcePathName);
      PathPlannerPath sourceToReefPath = PathPlannerPath.fromPathFile(sourceToReefPathName);
      Pose2d startPathEndPoint = startPath.getPathPoses().get(startPath.getPathPoses().size() - 1);
      Pose2d reefToSourceEndPoint =
          reefToSourcePath.getPathPoses().get(reefToSourcePath.getPathPoses().size() - 1);
      Pose2d sourceToReefEndPoint =
          sourceToReefPath.getPathPoses().get(sourceToReefPath.getPathPoses().size() - 1);

      Pose2d firstReefTarget;
      if (firstScoreLocation == ReefAlignLocation.LEFT) {
        firstReefTarget = FieldConstants.getNearestLeftBranch(startPathEndPoint);
      } else if (firstScoreLocation == ReefAlignLocation.RIGHT) {
        firstReefTarget = FieldConstants.getNearestRightBranch(startPathEndPoint);
      } else {
        firstReefTarget = FieldConstants.getNearestReefFace(startPathEndPoint);
      }

      Pose2d sourceTarget = FieldConstants.getNearestSource(reefToSourceEndPoint);

      Pose2d secondReefTarget;
      if (secondScoreLocation == ReefAlignLocation.LEFT) {
        secondReefTarget = FieldConstants.getNearestLeftBranch(sourceToReefEndPoint);
      } else if (secondScoreLocation == ReefAlignLocation.RIGHT) {
        secondReefTarget = FieldConstants.getNearestRightBranch(sourceToReefEndPoint);
      } else {
        secondReefTarget = FieldConstants.getNearestReefFace(sourceToReefEndPoint);
      }

      Command startToReefCommand = AutoBuilder.followPath(startPath);

      Command firstAlignCommand =
          AutoBuilder.pathfindThenFollowPath(
              new PathPlannerPath(
                  PathPlannerPath.waypointsFromPoses(startPathEndPoint, firstReefTarget),
                  AutoAlignConstants.PATH_CONSTRAINTS,
                  null,
                  new GoalEndState(0, firstReefTarget.getRotation())) {
                {
                  preventFlipping = true;
                }
              },
              AutoAlignConstants.PATH_CONSTRAINTS);

      Command reefToSourceCommand = AutoBuilder.followPath(reefToSourcePath);

      Command sourceAlignCommand =
          AutoBuilder.pathfindThenFollowPath(
              new PathPlannerPath(
                  PathPlannerPath.waypointsFromPoses(reefToSourceEndPoint, sourceTarget),
                  AutoAlignConstants.PATH_CONSTRAINTS,
                  null,
                  new GoalEndState(0, sourceTarget.getRotation())) {
                {
                  preventFlipping = true;
                }
              },
              AutoAlignConstants.PATH_CONSTRAINTS);

      Command sourceToReefCommand = AutoBuilder.followPath(sourceToReefPath);

      Command secondAlignCommand =
          AutoBuilder.pathfindThenFollowPath(
              new PathPlannerPath(
                  PathPlannerPath.waypointsFromPoses(sourceToReefEndPoint, secondReefTarget),
                  AutoAlignConstants.PATH_CONSTRAINTS,
                  null,
                  new GoalEndState(0, secondReefTarget.getRotation())) {
                {
                  preventFlipping = true;
                }
              },
              AutoAlignConstants.PATH_CONSTRAINTS);

      Commands.sequence(
              startToReefCommand,
              firstAlignCommand,
              GamePieceCommands.placeCoralCommand(elevator, coralManipulator, firstScoreHeight),
              reefToSourceCommand,
              sourceAlignCommand,
              Commands.waitSeconds(3),
              sourceToReefCommand,
              secondAlignCommand,
              GamePieceCommands.placeCoralCommand(elevator, coralManipulator, secondScoreHeight))
          .schedule();

    } catch (Exception e) {
      System.out.println("Unable to run auto");
      e.printStackTrace();
    }
  }
}
