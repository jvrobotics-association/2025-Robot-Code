package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralManipulatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class CoralManipulator extends SubsystemBase {
  private final CANrange coralSensor = new CANrange(CoralManipulatorConstants.CORAL_SENSOR, "rio");
  private final TalonFX leftMotor = new TalonFX(CoralManipulatorConstants.LEFT_MOTOR, "rio");
  private final TalonFX rightMotor = new TalonFX(CoralManipulatorConstants.RIGHT_MOTOR, "rio");

  private final CANrangeConfiguration coralSensorConfig;
  private final TalonFXConfiguration leftMotorConfig;
  private final TalonFXConfiguration rightMotorConfig;

  private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

  public CoralManipulator() {
    // Create the configs used to configure the devices in this mechanism
    coralSensorConfig = new CANrangeConfiguration();
    leftMotorConfig = new TalonFXConfiguration();
    rightMotorConfig = new TalonFXConfiguration();

    // Configure the distance sensor range and update frequency
    coralSensorConfig
        .ToFParams
        .withUpdateMode(UpdateModeValue.ShortRangeUserFreq)
        .withUpdateFrequency(Hertz.of(50));

    // Configure the distance center FOV
    coralSensorConfig
        .FovParams
        .withFOVRangeY(6.75)
        .withFOVRangeX(6.5)
        .withFOVCenterX(3.5)
        .withFOVCenterY(3.5);

    /*
     * Configure the distance sensor proximity config.
     *
     * - Threshold is the the distance at which the object will be considered detected.
     * - Hysteresis is the distance above/below the threshold to consider as detected. This is used
     *   to prevent bouncing between states when not desired.
     */
    coralSensorConfig.ProximityParams.withProximityThreshold(0.2).withProximityHysteresis(0.01);

    // Apply the distance sensor config, retry config apply up to 5 times, report if failure
    StatusCode coralSensorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      coralSensorStatus = coralSensor.getConfigurator().apply(coralSensorConfig);
      if (coralSensorStatus.isOK()) break;
    }
    if (!coralSensorStatus.isOK()) {
      System.out.println(
          "Could not apply coral sensor config, error code: " + coralSensorStatus.toString());
    }

    leftMotorConfig.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
    leftMotorConfig
        .MotorOutput
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive);
    leftMotorConfig
        .CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amp.of(20));
    leftMotorConfig
        .HardwareLimitSwitch
        .withForwardLimitEnable(false)
        .withForwardLimitRemoteCANrange(coralSensor)
        .withForwardLimitRemoteSensorID(CoralManipulatorConstants.CORAL_SENSOR)
        .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen);

    // Apply the left motor config, retry config apply up to 5 times, report if failure
    StatusCode leftMotorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      leftMotorStatus = leftMotor.getConfigurator().apply(leftMotorConfig);
      if (leftMotorStatus.isOK()) break;
    }
    if (!leftMotorStatus.isOK()) {
      System.out.println(
          "Could not apply left coral motor config, error code: " + leftMotorStatus.toString());
    }

    rightMotorConfig.Feedback.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
    rightMotorConfig
        .MotorOutput
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive);
    rightMotorConfig
        .CurrentLimits
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Amp.of(20));
    rightMotorConfig
        .HardwareLimitSwitch
        .withForwardLimitEnable(false)
        .withForwardLimitRemoteCANrange(coralSensor)
        .withForwardLimitRemoteSensorID(CoralManipulatorConstants.CORAL_SENSOR)
        .withForwardLimitType(ForwardLimitTypeValue.NormallyOpen);

    // Apply the right motor config, retry config apply up to 5 times, report if failure
    StatusCode rightMotorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      rightMotorStatus = rightMotor.getConfigurator().apply(rightMotorConfig);
      if (rightMotorStatus.isOK()) break;
    }
    if (!rightMotorStatus.isOK()) {
      System.out.println(
          "Could not apply right coral motor config, error code: " + rightMotorStatus.toString());
    }

    // Set the right motor to follow the left motor
    rightMotor.setControl(new StrictFollower(CoralManipulatorConstants.LEFT_MOTOR));
  }

  @Override
  public void periodic() {
    if (leftMotor.getForwardLimit(true).getValue() == ForwardLimitValue.ClosedToGround) {
      leftMotor.setControl(dutyCycle.withOutput(0.2));
    } else leftMotor.stopMotor();
  }

  @AutoLogOutput(key = "Coral Manipulator/Distance")
  public Distance getCoralSensorDistance() {
    return coralSensor.getDistance(true).getValue();
  }

  @AutoLogOutput(key = "Coral Manipulator/Detected")
  public boolean getCoralSensorDetected() {
    return coralSensor.getIsDetected(true).getValue();
  }

  @AutoLogOutput(key = "Coral Manipulator/Right Velocity")
  public AngularVelocity getRightVelocity() {
    return rightMotor.getVelocity(true).getValue();
  }

  @AutoLogOutput(key = "Coral Manipulator/Left Velocity")
  public AngularVelocity getLeftVelocity() {
    return leftMotor.getVelocity(true).getValue();
  }

  public void setVelocity(double velocity) {
    leftMotor.set(velocity);
  }

  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
