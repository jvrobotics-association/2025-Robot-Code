package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import java.util.Map;

public class ArmSubsystem extends SubsystemBase {
  private final SparkMax m_leftMotor = new SparkMax(ArmConstants.LEFT_MOTOR, MotorType.kBrushless);
  private final SparkMax m_rightMotor =
      new SparkMax(ArmConstants.RIGHT_MOTOR, MotorType.kBrushless);

  private final SparkMaxConfig m_leftMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig m_rightMotorConfig = new SparkMaxConfig();

  private final SparkClosedLoopController m_ClosedLoopController;
  private final SparkAbsoluteEncoder m_encoder;

  private Double targetPosition = null;

  private ShuffleboardLayout layout =
      Shuffleboard.getTab("Testbench")
          .getLayout("Arm Subsystem", BuiltInLayouts.kList)
          .withSize(2, 3)
          .withPosition(8, 0);

  public GenericEntry angle =
      layout
          .add("Angle", 0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .withProperties(
              Map.of("min", 0, "max", ArmConstants.POSITION_HOLDING_CUTOFF, "block increment", 1))
          .getEntry();

  public ArmSubsystem() {
    // Configure the left motor
    m_leftMotorConfig
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(25)
        .voltageCompensation(12)
        .smartCurrentLimit(20);
    m_leftMotorConfig
        .limitSwitch
        .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(false);
    m_leftMotorConfig.softLimit.reverseSoftLimit(265f).reverseSoftLimitEnabled(true);
    m_leftMotorConfig
        .absoluteEncoder
        .setSparkMaxDataPortConfig()
        .positionConversionFactor(ArmConstants.postionConversionFactor)
        .inverted(true)
        .zeroOffset(ArmConstants.zeroOffset);
    m_leftMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(ArmConstants.kP)
        .i(ArmConstants.kI)
        .d(ArmConstants.kD)
        .iZone(ArmConstants.kIz)
        .velocityFF(ArmConstants.kFF)
        .outputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput)
        .positionWrappingEnabled(true)
        .positionWrappingMinInput(-180)
        .positionWrappingMaxInput(180);

    // Configure the right motor, this is just in a follow mode to follow the left motor
    m_rightMotorConfig
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(25)
        .voltageCompensation(12)
        .smartCurrentLimit(20)
        .follow(ArmConstants.LEFT_MOTOR, true);
    m_rightMotorConfig
        .limitSwitch
        .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(false);

    // Save the settings to the controller flash so they persist
    m_leftMotor.configure(
        m_leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(
        m_rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Get the encoder for use
    m_encoder = m_leftMotor.getAbsoluteEncoder();

    // Get the closed loop motion controller to use
    m_ClosedLoopController = m_leftMotor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    super.periodic();

    if (targetPosition != null) {
      m_ClosedLoopController.setReference(
          targetPosition - ArmConstants.zeroOffset, ControlType.kPosition);

      // double feedForward = m_feedForward.calculate(targetPosition, 0.5, 0.5);
      // m_pidController.setReference(targetPosition, ControlType.kPosition, 0,
      // feedForward, ArbFFUnits.kVoltage);
    }

    SmartDashboard.putNumber("Degrees", getDegrees() - ArmConstants.zeroOffset);
    SmartDashboard.putNumber(
        "Target Position", (targetPosition == null ? 0 : getTargetPositionInDegrees()));
  }

  public void setTargetPosition(double degree) {
    if (degree > ArmConstants.MAXIMUM_ANGLE) {
      targetPosition = ArmConstants.MAXIMUM_ANGLE * -1;
    } else if (degree < ArmConstants.MINIMUM_ANGLE) {
      targetPosition = ArmConstants.MINIMUM_ANGLE * -1;
    } else {
      targetPosition = degree * -1;
    }
  }

  public double getTargetPositionInDegrees() {
    return targetPosition * -1;
  }

  public double getTargetPositionInDegreesNullSafe() {
    if (targetPosition == null) {
      return -1;
    } else return targetPosition * -1;
  }

  public void raiseArmOpenLoop(double speed) {
    targetPosition = null;
    m_leftMotor.set(-speed);
  }

  public void raiseArm() {
    if (targetPosition == null && getDegrees() < ArmConstants.POSITION_HOLDING_CUTOFF) {
      setTargetPosition(Math.round(getDegrees()));
    } else {
      if (getDegrees() >= ArmConstants.POSITION_HOLDING_CUTOFF) {
        targetPosition = null;
        m_leftMotor.set(-0.18);
      } else {
        setTargetPosition(getTargetPositionInDegrees() + 0.5);
      }
    }
  }

  public void lowerArm() {
    if (targetPosition == null && getDegrees() < ArmConstants.POSITION_HOLDING_CUTOFF) {
      setTargetPosition(Math.round(getDegrees()));
    } else {
      if (getDegrees() >= ArmConstants.POSITION_HOLDING_CUTOFF) {
        targetPosition = null;
        m_leftMotor.set(0.18);
      } else {
        setTargetPosition(getTargetPositionInDegrees() - 0.5);
      }
    }
  }

  public double getDegrees() {
    return (m_encoder.getPosition() - 360) * -1;
  }

  /** Stop the arm movement */
  public void stop() {
    targetPosition = null;
    m_leftMotor.stopMotor();
  }
}
