package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AlgaeManiplulatorConstants;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drive;

public class GamePieceCommands {

  public static Command placeCoralCommand(
      Elevator elevator, CoralManipulator coralManipulator, ElevatorHeight elevatorHeight) {
    return Commands.sequence(
        new MoveElevator(elevator, elevatorHeight),
        Commands.deadline(
            Commands.waitSeconds(0.8),
            Commands.runEnd(
                () -> coralManipulator.setSpeed(CoralManipulatorConstants.OUTPUT_SPEED),
                () -> coralManipulator.stopMotors(),
                coralManipulator)),
        new MoveElevator(elevator, ElevatorHeight.L1),
        Commands.runOnce(() -> elevator.stopElevator(), elevator));
  }

  public static Command placeCoralLeftCommand(
      Elevator elevator, CoralManipulator coralManipulator) {
    return Commands.deadline(
        Commands.waitSeconds(0.3),
        Commands.runEnd(
            () -> coralManipulator.outputLeft(),
            () -> coralManipulator.stopMotors(),
            coralManipulator));
  }

  public static Command placeCoralRightCommand(
      Elevator elevator, CoralManipulator coralManipulator) {
    return Commands.deadline(
        Commands.waitSeconds(0.3),
        Commands.runEnd(
            () -> coralManipulator.outputRight(),
            () -> coralManipulator.stopMotors(),
            coralManipulator));
  }

  public static Command collectAlgae(
      Drive drive,
      Elevator elevator,
      AlgaeManipulator algaeManipulator,
      ElevatorHeight elevatorHeight) {
    return Commands.sequence(
        new MoveElevator(elevator, elevatorHeight),
        Commands.runOnce(
            () -> algaeManipulator.setRotationPosition(AlgaeManiplulatorConstants.REEF_GRAB),
            algaeManipulator),
        Commands.waitSeconds(0.2),
        Commands.deadline(
            new IntakeAlgae(algaeManipulator),
            Commands.runEnd(
                () -> elevator.manuallyRaise(), () -> elevator.holdCurrentPosition(), elevator)),
        Commands.deadline(
            Commands.waitSeconds(0.5),
            DriveCommands.joystickDrive(drive, () -> true, () -> -0.1, () -> 0, () -> 0)),
        Commands.parallel(
            new MoveElevator(elevator, ElevatorHeight.L1),
            Commands.runOnce(
                () ->
                    algaeManipulator.setRotationPosition(AlgaeManiplulatorConstants.START_POSITION),
                algaeManipulator)));
  }
}
