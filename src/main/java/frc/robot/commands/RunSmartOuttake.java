// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunSmartOuttake extends Command {
  private final IntakeSubsystem m_subsystem;

  public RunSmartOuttake(IntakeSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.out(0.2);
  }

  @Override
  public boolean isFinished() {
    return m_subsystem.noteIsLoaded();
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopIntake();
  }
}
