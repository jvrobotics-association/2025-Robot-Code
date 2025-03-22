package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class Climber extends SubsystemBase {
  private static final Servo chuteServo = new Servo(ClimberConstants.CHUTE_SERVO);
  private static final Servo ratchetServo = new Servo(ClimberConstants.RATCHET_SERVO);
  private static final TalonFX rotationMotor = new TalonFX(ClimberConstants.ROTATION_MOTOR, "rio");
  private static final TalonFX winchMotor = new TalonFX(ClimberConstants.WINCH_MOTOR, "rio");

  // Create the motion request used for the rotation motor
  final MotionMagicTorqueCurrentFOC rotationRequest = new MotionMagicTorqueCurrentFOC(0);

  final DutyCycleOut m_manualRequest = new DutyCycleOut(0);

  public Climber() {
    TalonFXConfiguration rotationMotorConfig = new TalonFXConfiguration();
    TalonFXConfiguration winchMotorConfig = new TalonFXConfiguration();

    /*
     * Configure PID Slot Gains. The units when used with MotionMagicTorqueCurrentFOC is in Amps
     *
     * Tuning steps:
     *  1. Set all values to 0
     *  2. Using Pheonix Tuner X, configure kG until the arm holds its position. Start by increasing it until the
     *      elevator goes up and then back it off until it stops moving and holds in one spot.
     *  3. Set kA so that the Measured Acceleration matches the requested acceleration.
     */
    rotationMotorConfig
        .Slot0
        .withKP(1000)
        .withKI(0)
        .withKD(4)
        .withKS(0)
        .withKV(0.09)
        .withKA(0.04)
        .withKG(1.63)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    // Configure the software limit switch to help prevent the climber from going out of bounds and
    // causing damage
    rotationMotorConfig
        .SoftwareLimitSwitch
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(ClimberConstants.ROTATION_MAX)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(ClimberConstants.ROTATION_MIN);

    // Configure the motor to use coast mode so it can be moved by the winch
    rotationMotorConfig
        .MotorOutput
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);

    // Configure the current limits set when moving the climber
    // rotationMotorConfig
    //     .TorqueCurrent
    //     .withPeakForwardTorqueCurrent(Amps.of(20)) // Maximum amps when extending the climber
    //     .withPeakReverseTorqueCurrent(Amps.of(10)); // Maxiumum amps when retracting the climber

    // Absolute limit of the amps the motor can draw to help prevent brownout
    // rotationMotorConfig.CurrentLimits.withStatorCurrentLimit(Amps.of(30));

    // Configure target cruise velosity (rotations per second)
    rotationMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 7;

    // Configure max acceleration (rotations per second)
    // If this is set to the cruise velocity, it will take 1 second to get to that velocity. Adjust
    // accordingly.
    rotationMotorConfig.MotionMagic.MotionMagicAcceleration = 15;

    // Configure the maximum motion jerk (the extreme bursts) (rotations per second)
    rotationMotorConfig.MotionMagic.MotionMagicJerk = 150;

    // Retry config apply up to 5 times, report if failure
    StatusCode rotationMotorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      rotationMotorStatus = rotationMotor.getConfigurator().apply(rotationMotorConfig);
      if (rotationMotorStatus.isOK()) break;
    }
    if (!rotationMotorStatus.isOK()) {
      System.out.println(
          "Could not apply climber rotation motor config, error code: "
              + rotationMotorStatus.toString());
    }

    // Configure the motor to use coast mode so it can be moved by the winch
    winchMotorConfig
        .MotorOutput
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive);

    // Retry config apply up to 5 times, report if failure
    StatusCode winchMotorStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      winchMotorStatus = winchMotor.getConfigurator().apply(winchMotorConfig);
      if (winchMotorStatus.isOK()) break;
    }
    if (!winchMotorStatus.isOK()) {
      System.out.println(
          "Could not apply climber winch motor config, error code: " + winchMotorStatus.toString());
    }

    rotationMotor.setPosition(0);
    chuteServo.setPosition(0);
    ratchetServo.setPosition(ClimberConstants.RATCHET_SERVO_CLOSED);
  }

  @AutoLogOutput(key = "Climber/Position")
  public double getClimberPosition() {
    return rotationMotor.getPosition(true).getValue().magnitude();
  }

  @AutoLogOutput(key = "Climber/PID Position")
  public double getClimberPidPosition() {
    return rotationRequest.getPositionMeasure().magnitude();
  }

  public void releaseChute() {
    chuteServo.setPosition(ClimberConstants.CHUTE_SERVO_OPEN);
  }

  public void releaseRatchet() {
    ratchetServo.setPosition(ClimberConstants.RATCHET_SERVO_OPEN);
  }

  public void engageRatchet() {
    ratchetServo.setPosition(ClimberConstants.RATCHET_SERVO_CLOSED);
  }

  public void extendClimber() {
    winchMotor.stopMotor();
    rotationMotor.setControl(rotationRequest.withPosition(ClimberConstants.ROTATION_MAX));
  }

  public void extendWinch() {
    winchMotor.setControl(m_manualRequest.withOutput(ClimberConstants.EXTEND_SPEED));
  }

  public void retractClimber() {
    rotationMotor.stopMotor();
    winchMotor.setControl(m_manualRequest.withOutput(ClimberConstants.RETRACT_SPEED));
  }

  public void stopRotationMotor() {
    rotationMotor.stopMotor();
  }

  public void stopWinchMotor() {
    winchMotor.stopMotor();
  }

  public void stopClimber() {
    rotationMotor.stopMotor();
    winchMotor.stopMotor();
  }
}
