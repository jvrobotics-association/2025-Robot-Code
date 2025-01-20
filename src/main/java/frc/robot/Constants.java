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

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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

  public static final int BLINKIN_LED_CONTROLLER_PORT = 0;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorPanelPort = 1;
  }

  public static class SetpointConstants {
    public static final double farSpeakerShooterSpeed = 0.55;
    public static final int farSpeakerArmAngle = 45;
    public static final double closeSpeakerShooterSpeed = 0.53;
    public static final int closeSpeakerArmAngle = 32;
    public static final double ampShooterSpeed = 0.35;
  }

  public static class VisionConstants {
    public static final Transform3d LEFT_CAMERA_TO_CENTER =
        new Transform3d(
            // Define the left facing camera location on the robot in terms of X,Y,Z coordinates
            // Parameters are ordered X, Y, Z
            new Translation3d(
                Units.inchesToMeters(-10.23),
                Units.inchesToMeters(1.0),
                Units.inchesToMeters(17.72)),

            // Define the way the left facing camera is oriented on the robot
            // Parameters are ordered Roll, Pitch, Yaw
            new Rotation3d(
                Units.degreesToRadians(11.56),
                Units.degreesToRadians(-21),
                Units.degreesToRadians(146)));
  }

  public static class ShooterConstants {
    public static final int BOTTOM_MOTOR = 5;
    public static final int TOP_MOTOR = 6;
  }

  public static class IntakeConstants {
    public static final int INTAKE_MOTOR = 7;
  }

  public static class ClimberConstants {
    public static final int CLIMBER_MOTOR = 2;
    public static final int CLIMBER_ENCODER = 1;
    public static final int PH_ID = 9;
    public static final int FORWARD_CHANNEL = 14;
    public static final int REVERSE_CHANNEL = 15;
  }

  public static class ArmConstants {
    public static final int LEFT_MOTOR = 3;
    public static final int RIGHT_MOTOR = 4;

    public static final double kP = 0.04;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kIz = 0;
    public static final double kFF = 0;
    public static final double kMaxOutput = 0.35; // Down
    public static final double kMinOutput = -0.5; // Up
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;

    public static final double postionConversionFactor = 360;
    public static final double zeroOffset = 154.3;

    public static final double MINIMUM_ANGLE = 1;
    public static final double MAXIMUM_ANGLE = 80;

    public static final double POSITION_HOLDING_CUTOFF = 70;
  }
}
