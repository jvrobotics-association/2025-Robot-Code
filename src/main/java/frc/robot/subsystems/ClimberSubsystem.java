package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR);
  private final PneumaticHub m_pneumaticHub = new PneumaticHub(ClimberConstants.PH_ID);
  private final DoubleSolenoid m_doubleSolenoid =
      m_pneumaticHub.makeDoubleSolenoid(
          ClimberConstants.FORWARD_CHANNEL, ClimberConstants.REVERSE_CHANNEL);

  private final PositionTorqueCurrentFOC m_torquePosition;

  public ClimberSubsystem() {
    m_torquePosition =
        new PositionTorqueCurrentFOC(0)
            .withVelocity(0)
            .withFeedForward(0)
            .withSlot(0)
            .withOverrideCoastDurNeutral(false)
            .withLimitForwardMotion(false)
            .withLimitReverseMotion(false);
    m_pneumaticHub.enableCompressorAnalog(100, 110);
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 60; // An error of 1 rotations results in 40 amps output
    configs.Slot0.kD = 4; // A change of 1 rotation per second results in 2 amps output
    // Peak output of 100 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 100;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -100;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    climberMotor.getConfigurator().apply(configs);

    climberMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Climber Position", climberMotor.getRotorPosition().getValueAsDouble());
  }

  public void unlockBreak() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void lockBreak() {
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void extend() {
    climberMotor.setControl(m_torquePosition.withPosition(130));
  }

  public void retract() {
    climberMotor.setControl(m_torquePosition.withPosition(-3));
  }

  public void manualRetract() {
    climberMotor.set(-0.3);
  }

  public boolean isRetracted() {
    return climberMotor.getRotorPosition().getValueAsDouble() <= 3;
  }

  public boolean isExtended() {
    return climberMotor.getRotorPosition().getValueAsDouble() >= 128;
  }

  public void zeroPosition() {
    climberMotor.setPosition(0);
  }

  public void stopClimber() {
    climberMotor.stopMotor();
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }
}
