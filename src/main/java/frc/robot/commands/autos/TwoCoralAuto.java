package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefAlignLocation;
import frc.robot.commands.GamePieceCommands;
import frc.robot.commands.MoveElevator;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;

public class TwoCoralAuto extends Command {
  private final Elevator elevator;
  private final CoralManipulator coralManipulator;
  private final String firstFacePathName;
  private final ReefAlignLocation firstScoreLocation;
  private final ElevatorHeight firstScoreHeight;
  private final String firstSourcePathName;
  private final String secondFacePathName;
  private final ReefAlignLocation secondScoreLocation;
  private final ElevatorHeight secondScoreHeight;

  public TwoCoralAuto(
      Elevator elevator,
      CoralManipulator coralManipulator,
      String firstFacePathName,
      ReefAlignLocation firstScoreLocation,
      ElevatorHeight firstScoreHeight,
      String firstSourcePathName,
      String secondFacePathName,
      ReefAlignLocation secondScoreLocation,
      ElevatorHeight secondScoreHeight) {
    this.elevator = elevator;
    this.coralManipulator = coralManipulator;
    this.firstFacePathName = firstFacePathName;
    this.firstScoreLocation = firstScoreLocation;
    this.firstScoreHeight = firstScoreHeight;
    this.firstSourcePathName = firstSourcePathName;
    this.secondFacePathName = secondFacePathName;
    this.secondScoreLocation = secondScoreLocation;
    this.secondScoreHeight = secondScoreHeight;
  }

  @Override
  public void initialize() {
    try {
      PathPlannerPath firstFacePath = PathPlannerPath.fromPathFile(firstFacePathName);

      Command firstFacePathCommand = AutoBuilder.followPath(firstFacePath);

      Pose2d firstFacePathEndPoint =
          firstFacePath.getPathPoses().get(firstFacePath.getPathPoses().size() - 1);

      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
        firstFacePathEndPoint = FlippingUtil.flipFieldPose(firstFacePathEndPoint);
      }

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

      PathPlannerPath firstSourcePath = PathPlannerPath.fromPathFile(firstSourcePathName);

      Command firstSourcePathCommand = AutoBuilder.followPath(firstSourcePath);

      Pose2d firstSourcePathEndPoint =
          firstSourcePath.getPathPoses().get(firstSourcePath.getPathPoses().size() - 1);

      Pose2d firstSourceTarget = FieldConstants.getNearestSource(firstSourcePathEndPoint);

      Command firstSourceAlignCommand =
          AutoBuilder.followPath(
              new PathPlannerPath(
                  PathPlannerPath.waypointsFromPoses(firstSourcePathEndPoint, firstSourceTarget),
                  AutoAlignConstants.PATH_CONSTRAINTS,
                  null,
                  new GoalEndState(0, firstSourceTarget.getRotation())) {
                {
                  preventFlipping = true;
                }
              });

      PathPlannerPath secondFacePath = PathPlannerPath.fromPathFile(secondFacePathName);

      Command secondFacePathCommand = AutoBuilder.followPath(secondFacePath);

      Pose2d secondFacePathEndPoint =
          secondFacePath.getPathPoses().get(secondFacePath.getPathPoses().size() - 1);

      Pose2d secondScoreTarget;
      if (secondScoreLocation == ReefAlignLocation.LEFT) {
        secondScoreTarget = FieldConstants.getNearestLeftBranch(secondFacePathEndPoint);
      } else if (secondScoreLocation == ReefAlignLocation.RIGHT) {
        secondScoreTarget = FieldConstants.getNearestRightBranch(secondFacePathEndPoint);
      } else {
        secondScoreTarget = FieldConstants.getNearestReefFace(secondFacePathEndPoint);
      }

      Command secondFaceAlignCommand =
          AutoBuilder.followPath(
              new PathPlannerPath(
                  PathPlannerPath.waypointsFromPoses(secondFacePathEndPoint, secondScoreTarget),
                  AutoAlignConstants.PATH_CONSTRAINTS,
                  null,
                  new GoalEndState(0, secondScoreTarget.getRotation())) {
                {
                  preventFlipping = true;
                }
              });

      Commands.sequence(
              firstFacePathCommand,
              firstFaceAlignCommand,
              new MoveElevator(elevator, firstScoreHeight),
              Commands.deadline(
                  Commands.waitSeconds(0.7),
                  Commands.runEnd(
                      () -> coralManipulator.setSpeed(CoralManipulatorConstants.OUTPUT_SPEED),
                      () -> coralManipulator.stopMotors(),
                      coralManipulator)),
              Commands.parallel(
                  new MoveElevator(elevator, ElevatorHeight.L1), firstSourcePathCommand),
              Commands.runOnce(() -> elevator.stopElevator(), elevator),
              firstSourceAlignCommand,
              Commands.waitUntil(() -> coralManipulator.getCoralSensorDetected()),
              secondFacePathCommand,
              secondFaceAlignCommand,
              GamePieceCommands.placeCoralCommand(elevator, coralManipulator, secondScoreHeight))
          .schedule();

    } catch (Exception e) {
      System.out.println("Unable to run auto");
      e.printStackTrace();
    }
  }
}
