package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralManipulatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class CoralManipulator extends SubsystemBase {
  private final CANrange coralSensor = new CANrange(CoralManipulatorConstants.CORAL_SENSOR, "rio");
  private final TalonFX leftMotor =
      new TalonFX(CoralManipulatorConstants.LEFT_MOTOR, "rio");
  private final TalonFX rightMotor =
      new TalonFX(CoralManipulatorConstants.RIGHT_MOTOR, "rio");

  // private final SparkClosedLoopController leftVelocityController;
  // private final SparkClosedLoopController rightVelocityController;
  private final RelativeEncoder leftMotorEncoder;
  private final RelativeEncoder rightMotorEncoder;

  private final CANrangeConfiguration coralSensorConfig;
  private final TalonFXConfiguration leftMotorConfig;
  private final TalonFXConfiguration rightMotorConfig;

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

    /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    // leftVelocityController = leftMotor.getClosedLoopController();
    // rightVelocityController = rightMotor.getClosedLoopController();
    leftMotorEncoder = leftMotor.getEncoder();
    rightMotorEncoder = rightMotor.getEncoder();

    leftMotorConfig.voltageCompensation(12).smartCurrentLimit(20).idleMode(IdleMode.kBrake);

    rightMotorConfig
        .voltageCompensation(12)
        .smartCurrentLimit(20)
        .idleMode(IdleMode.kBrake)
        .follow(CoralManipulatorConstants.LEFT_MOTOR, true);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    // motorConfig
    //     .closedLoop
    //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //     .p(0.0001)
    //     .d(0)
    //     .velocityFF(1.0 / 917)
    //     .outputRange(-1, 1);

    // motorConfig
    //     .closedLoop
    //     .maxMotion
    //     .maxVelocity(3000)
    //     .maxAcceleration(6000)
    //     .allowedClosedLoopError(1);

    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    leftMotor.configure(
        leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(
        rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    if (getCoralSensorDetected()) {
      setVelocity(0.15);
    } else stopMotors();
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
  public double getRightVelocity() {
    return rightMotorEncoder.getVelocity();
  }

  @AutoLogOutput(key = "Coral Manipulator/Left Velocity")
  public double getLeftVelocity() {
    return leftMotorEncoder.getVelocity();
  }

  public void setVelocity(double velocity) {
    leftMotor.set(velocity);
    // leftVelocityController.setReference(velocity, ControlType.kMAXMotionVelocityControl);
    // rightVelocityController.setReference(-velocity, ControlType.kMAXMotionVelocityControl);
  }

  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }
}
