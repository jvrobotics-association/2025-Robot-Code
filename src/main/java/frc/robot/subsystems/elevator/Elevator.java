package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class Elevator extends SubsystemBase {

  private static final TalonFX elevatorMotor = new TalonFX(ElevatorConstants.ELEVATOR_MOTOR);

  @AutoLogOutput(key = "Elevator/Position")
  private static final PositionTorqueCurrentFOC m_torquePosition = new PositionTorqueCurrentFOC(0);

  public Elevator() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = 60; // An error of 1 rotations results in 40 amps output
    configs.Slot0.kD = 4; // A change of 1 rotation per second results in 2 amps output
    // Peak output of 100 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 100;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -100;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorMotor.getConfigurator().apply(configs);

    elevatorMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Elevator Position", elevatorMotor.getRotorPosition().getValueAsDouble());
  }

  public void raise() {
    elevatorMotor.setControl(m_torquePosition.withPosition(130));
  }

  public void lower() {
    elevatorMotor.setControl(m_torquePosition.withPosition(-3));
  }

  public static void manRaise() {
    if (elevatorMotor.getRotorPosition().getValueAsDouble() >= ElevatorConstants.ELEVATOR_MAX_HEIGHT) {
      maxHeight();
    } else {
    elevatorMotor.set(0.05);
    }
  }

  public static void manLower() {
    if (elevatorMotor.getRotorPosition().getValueAsDouble() <= ElevatorConstants.ELEVATOR_MIN_HEIGHT) {
      zero();
    } else {
      elevatorMotor.set(-0.05);
    }
  }

  public static void zero() {
    elevatorMotor.setControl(m_torquePosition.withPosition(ElevatorConstants.ELEVATOR_MIN_HEIGHT));
  }

  public static void maxHeight() {
    elevatorMotor.setControl(m_torquePosition.withPosition(ElevatorConstants.ELEVATOR_MAX_HEIGHT));
  }

  public static void stopElevator() {
    elevatorMotor.set(0);
  }
}
