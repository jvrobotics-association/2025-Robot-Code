// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax intakeMotor =
      new SparkMax(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);
  private final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kNoteColorTarget = new Color(0.533, 0.374, 0.093);
  private ColorMatchResult kColorMatchResult;

  public IntakeSubsystem() {
    intakeMotorConfig.idleMode(IdleMode.kBrake).voltageCompensation(12).smartCurrentLimit(15);
    intakeMotor.configure(
        intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_colorMatcher.addColorMatch(kNoteColorTarget);
    m_colorMatcher.setConfidenceThreshold(0.80);
    kColorMatchResult = m_colorMatcher.matchColor(m_colorSensor.getColor());
  }

  @Override
  public void periodic() {
    kColorMatchResult = m_colorMatcher.matchColor(m_colorSensor.getColor());
  }

  public boolean noteIsLoaded() {
    if (kColorMatchResult != null && kColorMatchResult.color == kNoteColorTarget) {
      return true;
    } else return false;
  }

  public void in(double speed) {
    intakeMotor.set(-speed);
  }

  public void out(double speed) {
    intakeMotor.set(speed);
  }

  public void stopIntake() {
    intakeMotor.stopMotor();
  }
}
