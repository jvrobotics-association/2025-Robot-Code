package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeManiplulatorConstants;

public class AlgaeManipulator extends SubsystemBase {
  private final SparkFlex rotationMotor =
      new SparkFlex(AlgaeManiplulatorConstants.ROTATION_MOTOR, MotorType.kBrushless);
  private final SparkMax grabberMotor =
      new SparkMax(AlgaeManiplulatorConstants.GRABBER_MOTOR, MotorType.kBrushless);
  private final SparkAbsoluteEncoder encoder;

  private final SparkFlexConfig rotationMotorConfig = new SparkFlexConfig();
  private final SparkMaxConfig grabberMotorConfig = new SparkMaxConfig();

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
        .outputRange(0, 1)
        .p(0)
        .i(0)
        .d(0)
        .iZone(0)
        .velocityFF(0);

    rotationMotor.configure(
        rotationMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = rotationMotor.getAbsoluteEncoder();
  }
}
