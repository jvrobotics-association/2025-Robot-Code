package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class Climber extends SubsystemBase {
  private static final TalonFX climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR, "rio");
  // private static final CANcoder climberEncoder = new CANcoder

  // Create the motor control request type and ensure it is set to respect configured limits
  final MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(0);

  final DutyCycleOut m_manualRequest = new DutyCycleOut(0);

  public Climber() {
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();

    // Configure the motor to use brake mode to help hold position when disabled
    motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    // Configure the current limits set when moving the elevator
    motorConfig
        .TorqueCurrent
        .withPeakForwardTorqueCurrent(Amps.of(40)) // Maximum amps when lifting the elevator
        .withPeakReverseTorqueCurrent(Amps.of(40)); // Maxiumum amps when lowering the elevator
  }

  @AutoLogOutput(key = "Climber/Position")
  public double getPosition() {
    return climberMotor.getPosition(true).getValue().magnitude();
  }

  public void manuallyExtend() {
    climberMotor.setControl(m_manualRequest.withOutput(ClimberConstants.CLIMBER_MANUAL_SPEED));
  }

  public void manuallyRetract() {
    climberMotor.setControl(m_manualRequest.withOutput(-ClimberConstants.CLIMBER_MANUAL_SPEED));
  }

  public void stopClimber() {
    climberMotor.stopMotor();
  }

  public void holdCurrentPosition() {
    climberMotor.setControl(m_request.withPosition(getPosition()).withSlot(3));
  }

  public void moveToPosition(double position) {
    climberMotor.setControl(m_request.withPosition(position).withSlot(2));
  }
}
