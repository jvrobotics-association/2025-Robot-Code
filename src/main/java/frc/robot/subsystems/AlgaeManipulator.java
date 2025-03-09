package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeManiplulatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class AlgaeManipulator extends SubsystemBase {
  private final SparkFlex rotationMotor =
      new SparkFlex(AlgaeManiplulatorConstants.ROTATION_MOTOR, MotorType.kBrushless);
  private final SparkMax grabberMotor =
      new SparkMax(AlgaeManiplulatorConstants.GRABBER_MOTOR, MotorType.kBrushless);

  private final SparkFlexConfig rotationMotorConfig = new SparkFlexConfig();
  private final SparkMaxConfig grabberMotorConfig = new SparkMaxConfig();

  private final SparkClosedLoopController rotationController;

  public AlgaeManipulator() {
    // Configure the rotation motor
    rotationMotorConfig
        .idleMode(IdleMode.kBrake)
        .closedLoopRampRate(0)
        .openLoopRampRate(0)
        .voltageCompensation(12)
        .smartCurrentLimit(30, 30);
    rotationMotorConfig
        .absoluteEncoder
        .setSparkMaxDataPortConfig()
        .inverted(true)
        .zeroOffset(AlgaeManiplulatorConstants.ENCODER_OFFSET)
        .positionConversionFactor(25);
    rotationMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .outputRange(-0.08, 0.4)
        .p(0.12)
        .i(0)
        .d(1)
        .iZone(0)
        .velocityFF(0.004);
    rotationMotorConfig
        .limitSwitch
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchEnabled(false);
    rotationMotorConfig
        .softLimit
        .reverseSoftLimit(AlgaeManiplulatorConstants.MIN_POSITION)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(AlgaeManiplulatorConstants.MAX_POSITION)
        .forwardSoftLimitEnabled(true);
    rotationMotor.configure(
        rotationMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Configure the grabber motor
    grabberMotorConfig
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(25)
        .voltageCompensation(12)
        .smartCurrentLimit(1, 5, 200);
    grabberMotor.configure(
        grabberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Get the closed loop controller that is used to control the position of the arm
    rotationController = rotationMotor.getClosedLoopController();

    setRotationPosition(AlgaeManiplulatorConstants.START_POSITION);
  }

  @AutoLogOutput(key = "Algae Manipulator/Position")
  public double getPosition() {
    return rotationMotor.getAbsoluteEncoder().getPosition();
  }

  @AutoLogOutput(key = "Algae Manipulator/Grabber Velocity")
  public double getGrabberVelocity() {
    return grabberMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "Algae Manipulator/Grabber Current")
  public double getGrabberCurrent() {
    return grabberMotor.getOutputCurrent();
  }

  public void setRotationPosition(double position) {
    rotationController.setReference(position, ControlType.kPosition);
  }

  public void setGrabberSpeed(double speed) {
    grabberMotor.set(speed);
  }

  public void stopAllMotors() {
    rotationMotor.stopMotor();
    grabberMotor.stopMotor();
  }

  public void stopRotation() {
    rotationMotor.stopMotor();
  }

  public void stopGrabber() {
    grabberMotor.stopMotor();
  }
}
