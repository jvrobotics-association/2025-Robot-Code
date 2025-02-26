package frc.robot.subsystems.GenericMotionProfiledSubsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.GenericMotionProfiledSubsystem.GenericMotionProfiledSubsystem.ProfileType;
import frc.robot.util.Util;
import frc.robot.util.drivers.Phoenix6Util;
import frc.robot.util.sim.PhysicsSim;
import java.util.ArrayList;
import java.util.List;

/**
 * Generic motion IO implementation for any motion mechanism using a TalonFX motor controller, an
 * optional follower motor, and an optional remote CANcoder encoder.
 */
public class GenericMotionProfiledSubsystemIOImpl implements GenericMotionProfiledSubsystemIO {

  private boolean mIsSim;
  private PhysicsSim mSim;

  // Constants data struct
  GenericMotionProfiledSubsystemConstants mConstants;

  // Local motor objects
  private TalonFX mMainMotor = null;
  private TalonFX mFollower = null;

  // Local motor configs
  private TalonFXConfiguration mMainConfig = null;
  private TalonFXConfiguration mFollowerConfig = null;

  // Local CanCoder object
  private CANcoder mCancoder = null;

  // Control

  // Maintain a copy of the Operational setpoint
  private double mOpSetpoint = 0.0;

  // Hold the latest encoder position, rotor velocity, and trajectory position
  private double mCurrPosition = 0.0;
  private double mCurrVelocity = 0.0;
  private double mCurrTrajectoryPosition = 0.0;
  private double mCurrSupplyCurrent = 0.0;

  // All the Talon StatusSignals of interest
  private final StatusSignal<Angle> mInternalPositionRotations;
  private final StatusSignal<AngularVelocity> mVelocityRps;
  private final StatusSignal<Double> mMainClosedLoopError;
  private final StatusSignal<Double> mMainClosedLoopReference;
  private final StatusSignal<Double> mMainClosedLoopReferenceSlope;
  private final List<StatusSignal<Voltage>> mAppliedVoltage =
      new ArrayList<StatusSignal<Voltage>>();
  private final List<StatusSignal<Current>> mSupplyCurrent = new ArrayList<StatusSignal<Current>>();
  private final List<StatusSignal<Current>> mTorqueCurrent = new ArrayList<StatusSignal<Current>>();
  private final List<StatusSignal<Temperature>> mTempCelsius =
      new ArrayList<StatusSignal<Temperature>>();
  private final StatusSignal<Angle> mEncoderAbsolutePositionRotations;
  private final StatusSignal<Angle> mEncoderRelativePositionRotations;

  // Use single-shot control requests, as robot loop will call continuously
  private final VoltageOut voltageControl =
      new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(0.0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0).withSlot(0);
  private final VelocityTorqueCurrentFOC velocityControl =
      new VelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0).withSlot(0);
  private final MotionMagicTorqueCurrentFOC motionMagicPositionControl =
      new MotionMagicTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0).withSlot(0);
  private final MotionMagicVelocityTorqueCurrentFOC motionMagicVelocityControl =
      new MotionMagicVelocityTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0).withSlot(0);
  private final CoastOut coastOut = new CoastOut();
  private final StaticBrake staticBrake = new StaticBrake();

  /*
   * Constructor
   */
  public GenericMotionProfiledSubsystemIOImpl(
      GenericMotionProfiledSubsystemConstants constants, boolean isSim) {

    mIsSim = isSim;

    // Local reference to constants
    mConstants = constants;

    // Instantiate the main TalonFX object
    mMainMotor =
        new TalonFX(mConstants.kLeaderMotor.getDeviceNumber(), mConstants.kLeaderMotor.getBus());

    // Get the motor configuration group and configure the main motor
    /*
     * Note: We can use the kMotorcConfig constants here for both REAL and SIM, because after
     * this class is instantiated, if needed, the Subsystem's constructor will pull the SIM
     * constants into LoggedTunableNumbers and update this config.
     */
    mMainConfig = mConstants.kMotorConfig;
    Phoenix6Util.applyAndCheckConfiguration(mMainMotor, mMainConfig);

    // If a follower motor has been specified, instantiate and configure it
    if (mConstants.kFollowMotor != null) {

      // Instantiate a follower motor object ...
      mFollower =
          new TalonFX(mConstants.kFollowMotor.getDeviceNumber(), mConstants.kFollowMotor.getBus());
      mFollowerConfig = mMainConfig;

      // Always disable soft limits in the Follower
      mFollowerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
      mFollowerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

      Phoenix6Util.applyAndCheckConfiguration(mFollower, mFollowerConfig);

      // ...and tie it to the main motor
      mFollower.setControl(
          new Follower(mConstants.kLeaderMotor.getDeviceNumber(), mConstants.kFollowerOpposesMain));
    }

    // If a remote CANcoder has been specified, instantiate and configure it
    if (mConstants.kCANcoder != null) {

      mCancoder =
          new CANcoder(mConstants.kCANcoder.getDeviceNumber(), mConstants.kCANcoder.getBus());
      Phoenix6Util.checkErrorAndRetry(
          () -> mCancoder.getConfigurator().apply(mConstants.kEncoderConfig));
    }

    // Set update frequencies for some basic output signals
    Phoenix6Util.checkErrorAndRetry(
        () -> mMainMotor.getBridgeOutput().setUpdateFrequency(200, 0.05));
    Phoenix6Util.checkErrorAndRetry(
        () -> mMainMotor.getFault_Hardware().setUpdateFrequency(4, 0.05));

    // Assign StatusSignals to our local variables
    mInternalPositionRotations = mMainMotor.getPosition();
    mVelocityRps = mMainMotor.getVelocity();
    mMainClosedLoopError = mMainMotor.getClosedLoopError();
    mMainClosedLoopReference = mMainMotor.getClosedLoopReference();
    mMainClosedLoopReferenceSlope = mMainMotor.getClosedLoopReferenceSlope();

    mAppliedVoltage.add(mMainMotor.getMotorVoltage());
    mSupplyCurrent.add(mMainMotor.getSupplyCurrent());
    mTorqueCurrent.add(mMainMotor.getTorqueCurrent());
    mTempCelsius.add(mMainMotor.getDeviceTemp());

    if (mConstants.kFollowMotor != null) {
      mAppliedVoltage.add(mFollower.getMotorVoltage());
      mSupplyCurrent.add(mFollower.getSupplyCurrent());
      mTorqueCurrent.add(mFollower.getTorqueCurrent());
      mTempCelsius.add(mFollower.getDeviceTemp());
    }

    if (mConstants.kCANcoder != null) {
      mEncoderAbsolutePositionRotations = mCancoder.getAbsolutePosition();
      mEncoderRelativePositionRotations = mCancoder.getPosition();
    } else {
      mEncoderAbsolutePositionRotations = null;
      mEncoderRelativePositionRotations = null;
    }

    // Set update frequencies for the StatusSignals of interest
    Phoenix6Util.checkErrorAndRetry(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                mInternalPositionRotations,
                mVelocityRps,
                mAppliedVoltage.get(0),
                mSupplyCurrent.get(0),
                mTorqueCurrent.get(0),
                mTempCelsius.get(0)));
    mMainMotor.optimizeBusUtilization(0, 1.0);

    if (mConstants.kFollowMotor != null) {
      Phoenix6Util.checkErrorAndRetry(
          () ->
              BaseStatusSignal.setUpdateFrequencyForAll(
                  100,
                  mAppliedVoltage.get(1),
                  mSupplyCurrent.get(1),
                  mTorqueCurrent.get(1),
                  mTempCelsius.get(1)));
      mFollower.optimizeBusUtilization(0, 1.0);
    }

    Phoenix6Util.checkErrorAndRetry(
        () ->
            BaseStatusSignal.setUpdateFrequencyForAll(
                200,
                mMainClosedLoopError,
                mMainClosedLoopReference,
                mMainClosedLoopReferenceSlope));

    if (mConstants.kCANcoder != null) {
      Phoenix6Util.checkErrorAndRetry(
          () ->
              BaseStatusSignal.setUpdateFrequencyForAll(
                  500, mEncoderAbsolutePositionRotations, mEncoderRelativePositionRotations));
      mCancoder.optimizeBusUtilization(0, 1.0);
    }

    if (mIsSim) {
      // Get the Physics Sim instance
      mSim = PhysicsSim.getInstance();

      // Create the appropriate sim profile
      switch (mConstants.SimType) {
        case ARM:
          mSim.addArmSim(
              mConstants.kName,
              mMainMotor,
              mCancoder,
              mConstants.kMotorSimConfig,
              mConstants.kArmSimConfig);
          break;
        case ELEVATOR:
          mSim.addElevatorSim(
              mConstants.kName,
              mMainMotor,
              mCancoder,
              mConstants.kMotorSimConfig,
              mConstants.kElevSimConfig);
          break;
        case ROLLER:
          mSim.addRollerSim(mConstants.kName, mMainMotor, mConstants.kMotorSimConfig);
          break;
        case NONE:
        default:
          break;
      }
    }

    // Send a neutral command to halt all motors.
    stop();
  }

  @Override
  public void updateInputs(GenericMotionProfiledIOInputs inputs) {

    if (mIsSim) {
      // Run the Sim to update all values
      mSim.run();
    }

    // Refresh all StatusSignals and signal result
    // refreshAll() is more efficient than doing each one individually
    inputs.leaderMotorConnected =
        BaseStatusSignal.refreshAll(
                mInternalPositionRotations,
                mVelocityRps,
                mAppliedVoltage.get(0),
                mSupplyCurrent.get(0),
                mTorqueCurrent.get(0),
                mTempCelsius.get(0))
            .isOK();

    if (mConstants.kFollowMotor != null) {
      inputs.followerMotorConnected =
          BaseStatusSignal.refreshAll(
                  mAppliedVoltage.get(1),
                  mSupplyCurrent.get(1),
                  mTorqueCurrent.get(1),
                  mTempCelsius.get(1))
              .isOK();
    }

    if (mConstants.kCANcoder != null) {
      inputs.CANcoderConnected =
          BaseStatusSignal.refreshAll(
                  mEncoderAbsolutePositionRotations, mEncoderRelativePositionRotations)
              .isOK();
    }

    // Due to an unfixed firmware error, certain closed-loop signals do not get
    // refreshed by refreshAll()
    // See: https://api.ctr-electronics.com/changelog#known-issues-20240209
    mMainClosedLoopError.refresh();
    mMainClosedLoopReference.refresh();
    mMainClosedLoopReferenceSlope.refresh();

    // Get motor position and velocity
    inputs.positionRot = mInternalPositionRotations.getValueAsDouble();
    inputs.velocityRps = mVelocityRps.getValueAsDouble();

    // Get closed loop stats
    inputs.errorRotations = mMainClosedLoopError.getValueAsDouble();
    inputs.activeTrajectoryPosition = mMainClosedLoopReference.getValueAsDouble();
    inputs.activeTrajectoryVelocity = mMainClosedLoopReferenceSlope.getValueAsDouble();

    // Get voltage, currents, and temperature for the main motor
    inputs.appliedVoltage[0] = mAppliedVoltage.get(0).getValueAsDouble();
    inputs.supplyCurrentAmps[0] = mSupplyCurrent.get(0).getValueAsDouble();
    inputs.torqueCurrentAmps[0] = mTorqueCurrent.get(0).getValueAsDouble();
    inputs.tempCelsius[0] = mTempCelsius.get(0).getValueAsDouble();

    if (mConstants.kFollowMotor != null) {
      // Get voltage, currents, and temperature for the follower motor
      inputs.appliedVoltage[1] = mAppliedVoltage.get(1).getValueAsDouble();
      inputs.supplyCurrentAmps[1] = mSupplyCurrent.get(1).getValueAsDouble();
      inputs.torqueCurrentAmps[1] = mTorqueCurrent.get(1).getValueAsDouble();
      inputs.tempCelsius[1] = mTempCelsius.get(1).getValueAsDouble();
    }

    if (mConstants.kCANcoder != null) {
      // Get encoder positions
      inputs.absoluteEncoderPositionRot = mEncoderAbsolutePositionRotations.getValueAsDouble();
      inputs.relativeEncoderPositionRot = mEncoderRelativePositionRotations.getValueAsDouble();
    }

    // Save specs of interest for API calls
    mCurrPosition = inputs.positionRot;
    mCurrVelocity = inputs.velocityRps;
    mCurrTrajectoryPosition = inputs.activeTrajectoryPosition;
    mCurrSupplyCurrent = inputs.supplyCurrentAmps[0];
  }

  /** Run Open Loop at the specified voltage */
  @Override
  public void runVoltage(double volts) {
    mMainMotor.setControl(voltageControl.withOutput(volts));
  }

  /** Run Open Loop at the specified current */
  @Override
  public void runCurrent(double amps, double maxDutyCycle) {
    mMainMotor.setControl(currentControl.withOutput(amps).withMaxAbsDutyCycle(maxDutyCycle));
  }

  /** Run Closed Loop to setpoint in rotations */
  @Override
  public void runToPosition(double position) {
    mMainMotor.setControl(positionControl.withPosition(position));
    mOpSetpoint = position;
  }

  /** Run Closed Loop to velocity in rotations/second */
  @Override
  public void runToVelocity(double velocity) {
    mMainMotor.setControl(velocityControl.withVelocity(velocity));
    mOpSetpoint = velocity;
  }

  /** Run Motion Magic to the specified setpoint */
  @Override
  public void runMotionMagicPosition(double setpoint) {
    mMainMotor.setControl(motionMagicPositionControl.withPosition(setpoint));
    mOpSetpoint = setpoint;
  }

  /** Run Motion Magic Velocity to the specified velocity */
  @Override
  public void runMotionMagicVelocity(double velocity) {
    mMainMotor.setControl(motionMagicVelocityControl.withVelocity(velocity));
  }

  /* Stop in Coast mode */
  @Override
  public void stopCoast() {
    mMainMotor.setControl(coastOut);
  }

  /* Stop in Brake mode */
  @Override
  public void stopBrake() {
    mMainMotor.setControl(staticBrake);
  }

  /* Stop in Open Loop */
  @Override
  public void stop() {
    runVoltage(0.0);
    mMainMotor.stopMotor();
  }

  /* Configure PID constants */
  @Override
  public void configurePID(double kP, double kI, double kD, boolean check) {
    mMainConfig.Slot0.kP = kP;
    mMainConfig.Slot0.kI = kI;
    mMainConfig.Slot0.kD = kD;

    if (check) {
      Phoenix6Util.applyAndCheckConfiguration(mMainMotor, mMainConfig);
    } else {
      mMainMotor.getConfigurator().apply(mMainConfig, mConstants.kCANTimeout);
    }
  }

  /* Configure Closed Loop constants */
  @Override
  public void configureGSVA(double kG, double kS, double kV, double kA, boolean check) {
    mMainConfig.Slot0.kG = kG;
    mMainConfig.Slot0.kS = kS;
    mMainConfig.Slot0.kV = kV;
    mMainConfig.Slot0.kA = kA;

    if (check) {
      Phoenix6Util.applyAndCheckConfiguration(mMainMotor, mMainConfig);
    } else {
      mMainMotor.getConfigurator().apply(mMainConfig, mConstants.kCANTimeout);
    }
  }

  /* Configure Motion constants */
  @Override
  public void configureMotion(double kCruise, double kAccel, double kJerk, boolean check) {
    mMainConfig.MotionMagic.MotionMagicCruiseVelocity = kCruise;
    mMainConfig.MotionMagic.MotionMagicAcceleration = kAccel;
    mMainConfig.MotionMagic.MotionMagicJerk = kJerk;

    if (check) {
      Phoenix6Util.applyAndCheckConfiguration(mMainMotor, mMainConfig);
    } else {
      mMainMotor.getConfigurator().apply(mMainConfig, mConstants.kCANTimeout);
    }
  }

  @Override
  // Return current operational setpoint
  public double getSetpoint() {
    return mOpSetpoint;
  }

  // Return latest rotor position
  @Override
  public double getPosition() {
    return mCurrPosition;
  }

  // Return latest trajectory position
  @Override
  public double getCurrTrajPos() {
    return mCurrTrajectoryPosition;
  }

  // Return latest rotor velocity
  @Override
  public double getVelocity() {
    return mCurrVelocity;
  }

  @Override
  /* Get current lead motor supply current) */
  public double getSupplyCurrent() {
    return mCurrSupplyCurrent;
  }

  @Override
  // Has current motion trajectory completed?
  public synchronized boolean atPosition(ProfileType profileType, double tolerance) {
    if (!(profileType instanceof ProfileType.POSITION)
        && !(profileType instanceof ProfileType.MM_POSITION)) {
      return false;
    }

    return Util.epsilonEquals(
        mCurrPosition, mOpSetpoint, Math.max(mConstants.kminTolerance, tolerance));
  }

  /**************************************************
   * Encoder homing and reset
   **************************************************/
  protected boolean mHasBeenZeroed = false;

  public synchronized void zeroSensors() {
    Phoenix6Util.checkErrorAndRetry(() -> mMainMotor.setPosition(0, mConstants.kCANTimeout));
    mHasBeenZeroed = true;
  }

  public synchronized boolean hasBeenZeroed() {
    return mHasBeenZeroed;
  }

  /**************************************************
   * Methods to update specific TalonFX configurations
   **************************************************/
  public void enableSoftLimits(boolean enable) {
    if (mConstants.kFollowMotor != null) {
      mFollowerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
      mFollowerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
      Phoenix6Util.applyAndCheckConfiguration(mFollower, mFollowerConfig);
    }

    mMainConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = enable;
    mMainConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = enable;
    Phoenix6Util.applyAndCheckConfiguration(mMainMotor, mMainConfig);
  }

  public void setNeutralMode(NeutralModeValue mode) {
    if (mConstants.kFollowMotor != null) {
      mFollowerConfig.MotorOutput.NeutralMode = mode;
      Phoenix6Util.applyAndCheckConfiguration(mFollower, mFollowerConfig);
    }
    mMainConfig.MotorOutput.NeutralMode = mode;
    Phoenix6Util.applyAndCheckConfiguration(mMainMotor, mMainConfig);
  }

  public synchronized void setSupplyCurrentLimit(double value, boolean enable, boolean check) {
    mMainConfig.CurrentLimits.SupplyCurrentLimit = value;
    mMainConfig.CurrentLimits.SupplyCurrentLimitEnable = enable;

    if (check) {
      Phoenix6Util.applyAndCheckConfiguration(mMainMotor, mMainConfig);
    } else {
      mMainMotor.getConfigurator().apply(mMainConfig);
    }
  }

  public void setStatorCurrentLimit(double currentLimit, boolean enable, boolean check) {
    mMainConfig.CurrentLimits.StatorCurrentLimit = currentLimit;
    mMainConfig.CurrentLimits.StatorCurrentLimitEnable = enable;

    if (check) {
      Phoenix6Util.applyAndCheckConfiguration(mMainMotor, mMainConfig);
    } else {
      mMainMotor.getConfigurator().apply(mMainConfig);
    }
  }

  public boolean checkDeviceConfiguration() {
    if (!Phoenix6Util.readAndVerifyConfiguration(mMainMotor, mMainConfig)) {
      return false;
    }
    if (mConstants.kFollowMotor != null) {
      if (!Phoenix6Util.readAndVerifyConfiguration(mFollower, mFollowerConfig)) {
        return false;
      }
    }
    return true;
  }
}
