package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Inch;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class Elevator extends SubsystemBase {
  private static final TalonFX motor = new TalonFX(ElevatorConstants.MOTOR);
  private static final CANcoder encoder = new CANcoder(ElevatorConstants.ENCODER);
  private static final CANrange distanceSensor = new CANrange(ElevatorConstants.DISTANCE_SENSOR);

  // Create the motor control request type and ensure it is set to respect configured limits
  final MotionMagicTorqueCurrentFOC m_request = new MotionMagicTorqueCurrentFOC(0)
      .withIgnoreHardwareLimits(false)
      .withLimitForwardMotion(true)
      .withLimitReverseMotion(true);

  // Keep a neutral out so we can disable the motor
  private final NeutralOut m_brake = new NeutralOut();

  public Elevator() {
    // Create the base configs that will be applied to the motor, encoder, and distance sensor
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    CANrangeConfiguration distanceSensorConfig = new CANrangeConfiguration();

    // Configure the encoder
    encoderConfig.MagnetSensor.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

    // Apply the encoder config, retry config apply up to 5 times, report if failure
    StatusCode encoderStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      encoderStatus = encoder.getConfigurator().apply(encoderConfig);
      if (encoderStatus.isOK()) break;
    }
    if (!encoderStatus.isOK()) {
      System.out.println("Could not apply configs, error code: " + encoderStatus.toString());
    }

    // Configure the distance sensor range and update frequency
    distanceSensorConfig.ToFParams
        .withUpdateMode(UpdateModeValue.ShortRangeUserFreq)
        .withUpdateFrequency(Hertz.of(50));
    
    // Configure the distance center FOV
    distanceSensorConfig.FovParams
        .withFOVRangeY(Degree.of(15))
        .withFOVRangeX(Degree.of(15))
        .withFOVCenterX(Degree.of(0))
        .withFOVCenterY(Degree.of(0));

    /*
     * Configure the distance sensor proximity config.
     * 
     * - Threshold is the the distance at which the object will be considered detected.
     * - Hysteresis is the distance above/below the threshold to consider as detected. This is used 
     *   to prevent bouncing between states when not desired.
     */
    distanceSensorConfig.ProximityParams
        .withProximityThreshold(Inch.of(1.0625))
        .withProximityHysteresis(Inch.of(0.0625));

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
        .withKG(0)
        .withKA(0)
        .withKS(0.1)
        .withKP(0)
        .withKI(0)
        .withKD(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        .withGravityType(GravityTypeValue.Elevator_Static);

    // Configure the software limit switch to help prevent the elevator from going out of bounds and causing damage
    motorConfig
        .SoftwareLimitSwitch
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ElevatorConstants.MAX_HEIGHT)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(ElevatorConstants.MIN_HEIGHT);
    
    // Configure the elevator motor to use the elevator encoder mounted on the main shaft for the elevator
    motorConfig.Feedback
        .withFeedbackRemoteSensorID(ElevatorConstants.ENCODER)
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withSensorToMechanismRatio(1)
        .withRotorToSensorRatio(25);
    
    // Configure the motor to use the CANRange to prevent driving the elevator down too far and to zero the position
    // of the elevator every time the elevator returns to the bottom. The limit switch is acticated when the CANRange detects
    // something within its proximity configuration above.
    motorConfig.HardwareLimitSwitch
        .withReverseLimitRemoteCANrange(distanceSensor)
        .withReverseLimitEnable(true)
        .withReverseLimitAutosetPositionEnable(true);

    // Configure the motor to use brake mode to help hold position when disabled
    motorConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    // Configure the current limits set when moving the elevator
    motorConfig
        .TorqueCurrent
        .withPeakForwardTorqueCurrent(Amps.of(40)) // Maximum amps when lifting the elevator
        .withPeakReverseTorqueCurrent(Amp.of(20)); // Maxiumum amps when lowering the elevator

    // Absolute limit of the amps the motor can draw to help prevent brownout
    motorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(50));

    // Configure target cruise velosity (rotations per second)
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 25;

    // Configure max acceleration (rotations per second)
    // If this is set to the cruise velocity, it will take 1 second to get to that velocity. Adjust
    // accordingly.
    motorConfig.MotionMagic.MotionMagicAcceleration = 50;

    // Configure the maximum motion jerk (the extreme bursts) (rotations per second)
    motorConfig.MotionMagic.MotionMagicJerk = 250;

    // Retry config apply up to 5 times, report if failure
    StatusCode motorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      motorStatus = motor.getConfigurator().apply(motorConfig);
      if (motorStatus.isOK()) break;
    }
    if (!motorStatus.isOK()) {
      System.out.println("Could not apply configs, error code: " + motorStatus.toString());
    }

    // Reset the position that the elevator currently is at to 0.
    // The physical elevator should be all the way down when this is set.
    motor.setPosition(0);
    encoder.setPosition(0);
  }

  @AutoLogOutput(key = "Elevator/Position")
  public Angle getPosition() {
    return motor.getPosition(true).getValue();
  }

  @AutoLogOutput(key = "Elevator/Acceleration")
  public AngularAcceleration getAcceleration() {
    return motor.getAcceleration(true).getValue();
  }

  public void moveToPosition(Angle position) {
    motor.setControl(m_request.withPosition(position));
  }

  public void manuallyRaise() {
    motor.set(0.7);
  }

  public void manuallyLower() {
    motor.set(-0.7);
  }

  public void stopElevator() {
    motor.setControl(m_brake);
  }
}