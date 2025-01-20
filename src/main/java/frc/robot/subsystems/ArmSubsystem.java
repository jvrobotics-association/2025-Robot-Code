package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax m_leftMotor = new SparkMax(ArmConstants.LEFT_MOTOR, MotorType.kBrushless);
    private final SparkMax m_rightMotor = new SparkMax(ArmConstants.RIGHT_MOTOR, MotorType.kBrushless);

    private final SparkMaxConfig m_leftMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig m_rightMotorConfig = new SparkMaxConfig();

    private final SparkClosedLoopController m_pidController;
    private final AbsoluteEncoder m_encoder;

    private Double targetPosition = null;

    private ShuffleboardLayout layout = Shuffleboard.getTab("Testbench")
            .getLayout("Arm Subsystem", BuiltInLayouts.kList)
            .withSize(2, 3)
            .withPosition(8, 0);

    public GenericEntry angle = layout.add("Angle", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", ArmConstants.POSITION_HOLDING_CUTOFF, "block increment", 1))
            .getEntry();

    public ArmSubsystem() {
      m_leftMotorConfig
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(25)
        .voltageCompensation(12)
        .smartCurrentLimit(20);
      m_leftMotorConfig.limitSwitch
        .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(false);
      m_rightMotorConfig
        .idleMode(IdleMode.kBrake)
        .openLoopRampRate(25)
        .voltageCompensation(12)
        .smartCurrentLimit(20)
        .follow(ArmConstants.LEFT_MOTOR, true);
        m_rightMotorConfig.limitSwitch
        .forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(false)
        .reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(false);


        // Configure the through bore encoder attacted to the left motor controller
        m_leftMotor.getAbsoluteEncoder(AbsoluteEncoder.Type.kDutyCycle)
                .setPositionConversionFactor(ArmConstants.postionConversionFactor);
        m_leftMotor.getAbsoluteEncoder(AbsoluteEncoder.Type.kDutyCycle).setInverted(true);
        m_leftMotor.getAbsoluteEncoder(AbsoluteEncoder.Type.kDutyCycle).setZeroOffset(ArmConstants.zeroOffset);

        // Configure soft limits
        // m_leftMotor.setSoftLimit(SoftLimitDirection.kForward, -60f);
        m_leftMotor.setSoftLimit(SoftLimitDirection.kReverse, 265f);
        // m_leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        // Get the encoder for use
        m_encoder = m_leftMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        // Get and configure the PID controller
        m_pidController = m_leftMotor.getPIDController();
        m_pidController.setFeedbackDevice(m_encoder);
        m_pidController.setP(ArmConstants.kP);
        m_pidController.setI(ArmConstants.kI);
        m_pidController.setD(ArmConstants.kD);
        m_pidController.setIZone(ArmConstants.kIz);
        m_pidController.setFF(ArmConstants.kFF);
        m_pidController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);
        m_pidController.setPositionPIDWrappingEnabled(true);
        m_pidController.setPositionPIDWrappingMinInput(-180);
        m_pidController.setPositionPIDWrappingMaxInput(180);

        // Save the settings to the controller flash so they persist
        m_leftMotor.burnFlash();
    }

    @Override
    public void periodic() {
        super.periodic();

        if (targetPosition != null) {
            m_pidController.setReference(targetPosition, ControlType.kPosition);

            // double feedForward = m_feedForward.calculate(targetPosition, 0.5, 0.5);
            // m_pidController.setReference(targetPosition, ControlType.kPosition, 0,
            // feedForward, ArbFFUnits.kVoltage);
        }

        SmartDashboard.putNumber("Degrees", getDegrees());
        SmartDashboard.putNumber("Target Position", (targetPosition == null ? 0 : getTargetPositionInDegrees()));
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

    /**
     * Stop the arm movement
     */
    public void stop() {
        targetPosition = null;
        m_leftMotor.stopMotor();
    }
}