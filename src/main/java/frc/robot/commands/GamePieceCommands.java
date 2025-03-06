package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.Constants.ElevatorConstants.ReefLevel;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;

public class GamePieceCommands {

  public static Command placeCoralCommand(
      Elevator elevator, CoralManipulator coralManipulator, ReefLevel reefLevel) {
    return Commands.sequence(
        Commands.runOnce(() -> elevator.moveToPosition(reefLevel.height)),
        Commands.waitSeconds(0.1),
        Commands.race(
            Commands.runEnd(
                () -> coralManipulator.setSpeed(CoralManipulatorConstants.OUTPUT_SPEED),
                () -> coralManipulator.stopMotors(),
                coralManipulator),
            Commands.waitSeconds(0.8)),
        Commands.runOnce(() -> elevator.moveToPosition(ReefLevel.L1.height), elevator));
  }
}
