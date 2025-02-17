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

import static edu.wpi.first.units.Units.Radian;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class OperatorConstants {

  }

  public static class CoralManipulatorConstants {
    public static final int LEFT_MOTOR = 1;
    public static final int RIGHT_MOTOR = 2;
    public static final int CORAL_SENSOR = 3;
  }

  public static class AlgaeManiplulatorConstants {
    public static final int ROTATION_MOTOR = 4;
    public static final int ROTATION_ENCODER = 5;
    public static final int GRABBER_MOTOR = 6;

    // Configure the setpoints for the various positions the algae arm should move to
    public static final double START_POSITION = 0;
    public static final double GROUND_PICKUP = 0;
    public static final double CORAL_STATION_GRAB = 0;
  }

  public static class ElevatorConstants {
    public static final int MOTOR = 7;
    public static final int ENCODER = 8;
    public static final int DISTANCE_SENSOR = 9;

    // Set the absolute limits for the elevator motion
    public static final Angle MIN_HEIGHT = Radian.of(10);
    public static final Angle MAX_HEIGHT = Radian.of(676);

    // Configure the setpoints for the various positions the elevator should move to for coral game pieces
    public static final Angle L1_CORAL_POSITION = Radian.of(10);
    public static final Angle L2_CORAL_POSITION = Radian.of(200);
    public static final Angle L3_CORAL_POSITION = Radian.of(450);
    public static final Angle L4_CORAL_POSITION = Radian.of(670);

    // Configure the setpoints for the various positions the elevator should move to for algae game pieces
    public static final Angle L2_ALGAE_POSITION = Radian.of(220);
    public static final Angle L3_ALGAE_POSITION = Radian.of(470);
  }

  public static class ClimberConstants {
    public static final int CLIMBER_MOTOR = 10;
  }
}
