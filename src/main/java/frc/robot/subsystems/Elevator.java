package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class Elevator extends SubsystemBase {
  private static final TalonFX motor = new TalonFX(ElevatorConstants.MOTOR, "rio");
  private static final CANcoder encoder = new CANcoder(ElevatorConstants.ENCODER, "rio");

  // Create the motor control request type and ensure it is set to respect configured limits
  final MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(0);

  final DutyCycleOut m_manualRequest = new DutyCycleOut(0);

  public Elevator() {
    // Create the base configs that will be applied to the motor, encoder, and distance sensor
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

    // Configure the encoder
    encoderConfig.MagnetSensor.withSensorDirection(SensorDirectionValue.Clockwise_Positive);

    // Apply the encoder config, retry config apply up to 5 times, report if failure
    StatusCode encoderStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      encoderStatus = encoder.getConfigurator().apply(encoderConfig);
      if (encoderStatus.isOK()) break;
    }
    if (!encoderStatus.isOK()) {
      System.out.println("Could not apply encoder config, error code: " + encoderStatus.toString());
    }

    /*
     * Configure PID Slot Gains. The units when used with MotionMagicTorqueCurrentFOC is in Amps
     *
     * Tuning steps:
     *  1. Set all values to 0
     *  2. Using Pheonix Tuner X, configure kG until the elevator holds its position. Start by increasing it until the
     *      elevator goes up and then back it off until it stops moving and holds in one spot.
     *  3. Set kA so that the Measured Acceleration matches the requested acceleration.
     */
    motorConfig
        .Slot0
        .withKP(1000)
        .withKI(0)
        .withKD(40)
        .withKS(10)
        .withKV(0)
        .withKA(0)
        .withKG(5.99)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        .withGravityType(GravityTypeValue.Elevator_Static);

    // Configure Gain slot 1 used for manual control of the elevator
    motorConfig
        .Slot1
        .withKP(0)
        .withKI(0)
        .withKD(0)
        .withKS(0)
        .withKV(0)
        .withKA(0)
        .withKG(12.5)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        .withGravityType(GravityTypeValue.Elevator_Static);

    // Configure the software limit switch to help prevent the elevator from going out of bounds and
    // causing damage
    motorConfig
        .SoftwareLimitSwitch
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ElevatorConstants.MAX_HEIGHT)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(ElevatorConstants.MIN_HEIGHT);

    // Configure the elevator motor to use the elevator encoder mounted on the main shaft for the
    // elevator
    motorConfig
        .Feedback
        .withFeedbackRemoteSensorID(ElevatorConstants.ENCODER)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withSensorToMechanismRatio(1)
        .withRotorToSensorRatio(25);

    // Configure the motor to use brake mode to help hold position when disabled
    motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    // Configure the current limits set when moving the elevator
    motorConfig
        .TorqueCurrent
        .withPeakForwardTorqueCurrent(Amps.of(40)) // Maximum amps when lifting the elevator
        .withPeakReverseTorqueCurrent(Amps.of(40)); // Maxiumum amps when lowering the elevator

    // Absolute limit of the amps the motor can draw to help prevent brownout
    motorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(50));

    // Configure target cruise velosity (rotations per second)
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 5;

    // Configure max acceleration (rotations per second)
    // If this is set to the cruise velocity, it will take 1 second to get to that velocity. Adjust
    // accordingly.
    motorConfig.MotionMagic.MotionMagicAcceleration = 10;

    // Configure the maximum motion jerk (the extreme bursts) (rotations per second)
    motorConfig.MotionMagic.MotionMagicJerk = 25;

    // Retry config apply up to 5 times, report if failure
    StatusCode motorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      motorStatus = motor.getConfigurator().apply(motorConfig);
      if (motorStatus.isOK()) break;
    }
    if (!motorStatus.isOK()) {
      System.out.println(
          "Could not apply elevator motor config, error code: " + motorStatus.toString());
    }

    // Reset the position that the elevator currently is at to 0.
    // The physical elevator should be all the way down when this is set.
    motor.setPosition(0);
    encoder.setPosition(0);
  }

  @AutoLogOutput(key = "Elevator/Position")
  public double getPosition() {
    return motor.getPosition(true).getValue().magnitude();
  }

  @AutoLogOutput(key = "Elevator/PID Position")
  public double getPidPosition() {
    return m_request.getPositionMeasure().magnitude();
  }

  @AutoLogOutput(key = "Elevator/Acceleration")
  public AngularAcceleration getAcceleration() {
    return motor.getAcceleration(true).getValue();
  }

  public void moveToPosition(double position) {
    motor.setControl(m_request.withPosition(position).withSlot(0));
  }

  public void manuallyRaise() {
    motor.setControl(m_manualRequest.withOutput(ElevatorConstants.MANUAL_SPEED));
  }

  public void manuallyLower() {
    motor.setControl(m_manualRequest.withOutput(-ElevatorConstants.MANUAL_SPEED));
  }

  public void holdCurrentPosition() {
    motor.setControl(m_request.withPosition(getPosition()).withSlot(1));
  }

  public void stopElevator() {
    motor.setControl(m_manualRequest.withOutput(0));
  }
}
