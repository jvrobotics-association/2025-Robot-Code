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
        .openLoopRampRate(25)
        .voltageCompensation(12)
        .smartCurrentLimit(20);
    rotationMotorConfig
        .absoluteEncoder
        .setSparkMaxDataPortConfig()
        .inverted(true)
        .zeroOffset(AlgaeManiplulatorConstants.ENCODER_OFFSET);
    rotationMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .outputRange(-0.5, 0.5)
        .p(0.04)
        .i(0)
        .d(0.001)
        .iZone(0)
        .velocityFF(0);
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
        .smartCurrentLimit(5, 10, 10);
    grabberMotor.configure(
        grabberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Get the closed loop controller that is used to control the position of the arm
    rotationController = rotationMotor.getClosedLoopController();
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
    rotationController.setReference(position, ControlType.kMAXMotionPositionControl);
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
