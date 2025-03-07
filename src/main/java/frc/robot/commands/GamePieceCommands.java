package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AlgaeManiplulatorConstants;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;
import frc.robot.subsystems.AlgaeManipulator;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;

public class GamePieceCommands {

  public static Command placeCoralCommand(
      Elevator elevator, CoralManipulator coralManipulator, ElevatorHeight elevatorHeight) {
    return Commands.sequence(
        new MoveElevator(elevator, elevatorHeight),
        Commands.waitSeconds(0.1),
        Commands.deadline(
            Commands.waitSeconds(0.8),
            Commands.runEnd(
                () -> coralManipulator.setSpeed(CoralManipulatorConstants.OUTPUT_SPEED),
                () -> coralManipulator.stopMotors(),
                coralManipulator)),
        Commands.runOnce(() -> elevator.moveToPosition(ElevatorHeight.L1.height), elevator));
  }

  public static Command collectAlgae(
      Elevator elevator, AlgaeManipulator algaeManipulator, ElevatorHeight elevatorHeight) {
    return Commands.sequence(
        Commands.parallel(
            new MoveElevator(elevator, elevatorHeight),
            Commands.runOnce(
                () ->
                    algaeManipulator.setRotationPosition(
                        AlgaeManiplulatorConstants.CORAL_STATION_GRAB),
                algaeManipulator)),
        Commands.waitSeconds(0.1),
        Commands.deadline(
            new IntakeAlgae(algaeManipulator),
            Commands.runEnd(
                () -> elevator.manuallyRaise(), () -> elevator.holdCurrentPosition(), elevator)));
  }
}
