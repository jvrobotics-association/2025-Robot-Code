// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX bottomShooterMotor;
  private final TalonFX topShooterMotor;

  public ShooterSubsystem() {
    bottomShooterMotor = new TalonFX(ShooterConstants.BOTTOM_MOTOR);
    topShooterMotor = new TalonFX(ShooterConstants.TOP_MOTOR);

    var currentLimits =
        new CurrentLimitsConfigs().withStatorCurrentLimit(40).withStatorCurrentLimitEnable(true);

    bottomShooterMotor.getConfigurator().apply(currentLimits);

    bottomShooterMotor.setNeutralMode(NeutralModeValue.Brake);
    topShooterMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void shoot(double speed) {
    topShooterMotor.set(speed);
    bottomShooterMotor.set(speed);
  }

  public void stopShooter() {
    bottomShooterMotor.stopMotor();
    topShooterMotor.stopMotor();
  }
}
