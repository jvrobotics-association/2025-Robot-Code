package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeManiplulatorConstants;
import org.littletonrobotics.junction.AutoLogOutput;

public class AlgaeGrabber extends SubsystemBase {
  private final SparkMax grabberMotor =
      new SparkMax(AlgaeManiplulatorConstants.GRABBER_MOTOR, MotorType.kBrushless);

  private final SparkMaxConfig grabberMotorConfig = new SparkMaxConfig();

  public AlgaeGrabber() {
    // Configure the grabber motor
    grabberMotorConfig
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(1, 5, 200);
    grabberMotor.configure(
        grabberMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @AutoLogOutput(key = "Algae Manipulator/Grabber Velocity")
  public double getGrabberVelocity() {
    return grabberMotor.getEncoder().getVelocity();
  }

  @AutoLogOutput(key = "Algae Manipulator/Grabber Current")
  public double getGrabberCurrent() {
    return grabberMotor.getOutputCurrent();
  }

  public void setGrabberSpeed(double speed) {
    grabberMotor.set(speed);
  }

  public void stopGrabber() {
    grabberMotor.stopMotor();
  }
}
