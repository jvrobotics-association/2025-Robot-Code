package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climber;

public class ClimberCommands {

  public static Command extendClimber(Climber climber) {
    return Commands.sequence(
        Commands.deadline(
            Commands.waitSeconds(0.1),
            Commands.runOnce(
                () -> {
                  climber.releaseRatchet();
                  climber.releaseChute();
                },
                climber)),
        new ExtendClimber(climber));
  }

  public static Command retractClimber(Climber climber) {
    return Commands.sequence(
        Commands.deadline(
            Commands.waitSeconds(0.1), Commands.runOnce(() -> climber.releaseRatchet(), climber)),
        new RetractClimber(climber));
  }
}
