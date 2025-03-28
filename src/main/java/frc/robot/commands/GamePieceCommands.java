package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AlgaeManiplulatorConstants;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drive;

public class GamePieceCommands {

  public static Command placeCoralCommand(
      Elevator elevator, CoralManipulator coralManipulator, ElevatorHeight elevatorHeight) {
    return Commands.sequence(
        new MoveElevator(elevator, elevatorHeight),
        Commands.deadline(
            Commands.waitSeconds(1),
            Commands.runEnd(
                () -> coralManipulator.setSpeed(CoralManipulatorConstants.OUTPUT_SPEED),
                () -> coralManipulator.stopMotors(),
                coralManipulator)),
        new MoveElevator(elevator, ElevatorHeight.L1),
        Commands.runOnce(() -> elevator.stopElevator(), elevator));
  }

  public static Command placeCoralL1LeftCommand(
      Elevator elevator, CoralManipulator coralManipulator) {
    return Commands.deadline(
        Commands.waitSeconds(0.4),
        Commands.runEnd(
            () -> coralManipulator.outputLeft(),
            () -> coralManipulator.stopMotors(),
            coralManipulator));
  }

  public static Command placeCoralL1RightCommand(
      Elevator elevator, CoralManipulator coralManipulator) {
    return Commands.deadline(
        Commands.waitSeconds(0.4),
        Commands.runEnd(
            () -> coralManipulator.outputRight(),
            () -> coralManipulator.stopMotors(),
            coralManipulator));
  }

  public static Command collectAlgae(
      Drive drive,
      Elevator elevator,
      AlgaeArm algaeManipulator,
      AlgaeGrabber algaeGrabber,
      ElevatorHeight elevatorHeight) {
    return Commands.sequence(
        new MoveElevator(elevator, elevatorHeight),
        Commands.deadline(
            new IntakeAlgae(algaeGrabber),
            Commands.runOnce(
                () -> algaeManipulator.setRotationPosition(AlgaeManiplulatorConstants.REEF_GRAB),
                algaeManipulator)),
        new MoveElevator(elevator, elevatorHeight.height + 0.25),
        Commands.deadline(
            Commands.waitSeconds(0.3),
            DriveCommands.joystickDrive(drive, () -> true, () -> 1, () -> -0.8, () -> 0, () -> 0)),
        Commands.deadline(Commands.waitSeconds(0.02), Commands.runOnce(() -> drive.stop(), drive)),
        Commands.deadline(
            Commands.waitSeconds(0.02),
            Commands.runOnce(
                () ->
                    algaeManipulator.setRotationPosition(AlgaeManiplulatorConstants.START_POSITION),
                algaeManipulator)),
        new MoveElevator(elevator, ElevatorHeight.L1));
  }

  public static Command scoreAlgae(
      Elevator elevator, AlgaeArm algaeManipulator, AlgaeGrabber algaeGrabber) {
    return Commands.sequence(
        new MoveElevator(elevator, ElevatorHeight.ALGAE_SCORE),
        Commands.deadline(
            Commands.waitSeconds(0.5),
            Commands.runEnd(
                () -> algaeGrabber.setGrabberSpeed(AlgaeManiplulatorConstants.GRABBER_SCORE_SPEED),
                () -> algaeGrabber.stopGrabber(),
                algaeManipulator)),
        new MoveElevator(elevator, ElevatorHeight.L1));
  }

  public static Command resetAlgaeManipulator(
      AlgaeArm algaeManipulator, AlgaeGrabber algaeGrabber) {
    return Commands.deadline(
        Commands.waitSeconds(0.02),
        Commands.runOnce(
            () -> algaeManipulator.setRotationPosition(AlgaeManiplulatorConstants.START_POSITION),
            algaeManipulator),
        Commands.runOnce(() -> algaeGrabber.stopGrabber(), algaeGrabber));
  }

  public static Command launchAlgae(
      Elevator elevator, AlgaeArm algaeManipulator, AlgaeGrabber algaeGrabber) {
    return Commands.sequence(
        Commands.parallel(
            new MoveElevator(elevator, ElevatorHeight.MAX_HEIGHT),
            Commands.sequence(
                Commands.waitSeconds(0.75),
                Commands.parallel(
                    Commands.runOnce(
                        () ->
                            algaeManipulator.setRotationPosition(
                                AlgaeManiplulatorConstants.HORIZONTAL),
                        algaeManipulator),
                    Commands.sequence(
                        Commands.waitSeconds(0.1),
                        Commands.deadline(
                            Commands.waitSeconds(1),
                            Commands.runEnd(
                                () ->
                                    algaeGrabber.setGrabberSpeed(
                                        AlgaeManiplulatorConstants.GRABBER_SCORE_SPEED),
                                () -> algaeGrabber.stopGrabber(),
                                algaeGrabber)))))),
        Commands.deadline(
            Commands.waitSeconds(0.02),
            Commands.runOnce(
                () ->
                    algaeManipulator.setRotationPosition(AlgaeManiplulatorConstants.START_POSITION),
                algaeManipulator),
            Commands.runOnce(() -> algaeGrabber.stopGrabber(), algaeGrabber)),
        new MoveElevator(elevator, ElevatorHeight.L1));
  }

  public static Command kyra(
      Elevator elevator, AlgaeArm algaeManipulator, AlgaeGrabber algaeGrabber) {
    return Commands.sequence(
        Commands.runOnce(
            () -> algaeManipulator.setRotationPosition(AlgaeManiplulatorConstants.HORIZONTAL),
            algaeManipulator),
        Commands.waitSeconds(0.25),
        Commands.runOnce(
            () -> algaeManipulator.setRotationPosition(AlgaeManiplulatorConstants.START_POSITION),
            algaeManipulator));
  }
}
