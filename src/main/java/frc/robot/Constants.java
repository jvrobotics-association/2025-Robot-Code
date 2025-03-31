// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final int PDH = 1;
  public static final int CANIDLE = 20;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class OperatorConstants {}

  public static class DriveConstants {
    // Absolute maximum speed is 4.292
    public static final double MAX_SPEED = 4;
  }

  public static class AutoAlignConstants {
    public static final PathConstraints PATH_CONSTRAINTS =
        new PathConstraints(
            LinearVelocity.ofBaseUnits(1.3, MetersPerSecond),
            LinearAcceleration.ofBaseUnits(1.5, MetersPerSecondPerSecond),
            AngularVelocity.ofBaseUnits(540, DegreesPerSecond),
            AngularAcceleration.ofBaseUnits(720, DegreesPerSecondPerSecond));
  }

  public static class CoralManipulatorConstants {
    public static final int LEFT_MOTOR = 2;
    public static final int RIGHT_MOTOR = 3;
    public static final int CORAL_SENSOR = 4;

    public static final double INTAKE_SPEED = 0.085;
    public static final double OUTPUT_SPEED = 0.7;
    public static final double L1_SLOW = 0.075;
    public static final double L1_FAST = 0.5;
  }

  public static class AlgaeManiplulatorConstants {
    public static final int ROTATION_MOTOR = 5;
    public static final int GRABBER_MOTOR = 6;

    public static final double ENCODER_OFFSET = 0.423;
    public static final double MIN_POSITION = 0.2;
    public static final double MAX_POSITION = 12.9;
    public static final double GRABBER_INTAKE_SPEED = 0.44;
    public static final double GRABBER_SCORE_SPEED = -1;

    // Configure the setpoints for the various positions the algae arm should move to
    public static final double START_POSITION = 0.2;
    public static final double GROUND_PICKUP = 0;
    public static final double REEF_GRAB = 4;
    public static final double HORIZONTAL = 8;
  }

  public static class ElevatorConstants {
    public static final int MOTOR = 7;
    public static final int ENCODER = 8;

    // Set the absolute limits for the elevator motion
    public static final double MIN_HEIGHT = 0.045;
    public static final double MAX_HEIGHT = 4.33;
    public static final double MANUAL_SPEED = 0.50;
    public static final double AUTO_SPEED = 50;

    // Configure the setpoints for the various positions the elevator should move to for coral game
    // pieces
    public enum ElevatorHeight {
      L1(ElevatorConstants.MIN_HEIGHT),
      L2(0.468),
      L3(1.915),
      L4(4.25),
      L2_ALGAE(1.415),
      L3_ALGAE(2.835),
      ALGAE_SCORE(0.192),
      MAX_HEIGHT(4.30);

      ElevatorHeight(double height) {
        this.height = height;
      }

      public final double height;
    }

    // Configure the setpoints for the various positions the elevator should move to for algae game
    // pieces
    public static final double ALGAE_SCORE_POSITION = 0.3;
  }

  public static class ClimberConstants {
    public static final int CHUTE_SERVO = 0;
    public static final int RATCHET_SERVO = 1;
    public static final int ROTATION_MOTOR = 9;
    public static final int WINCH_MOTOR = 10;

    public static final double ROTATION_MIN = 0.1;
    public static final double ROTATION_MAX = 1.315;

    public static final double RETRACT_SPEED = 0.5;
    public static final double EXTEND_SPEED = -0.50;

    public static final double CHUTE_SERVO_OPEN = 0.3;
    public static final double CHUTE_SERVO_CLOSED = 0;
    public static final double RATCHET_SERVO_OPEN = 0.045;
    public static final double RATCHET_SERVO_CLOSED = 0;
  }
}
