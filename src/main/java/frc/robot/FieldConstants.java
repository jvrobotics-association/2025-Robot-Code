package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  // Set the April Tag field to use
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // The length of the field in inches
  public static final double fieldLength = Units.inchesToMeters(690.876);

  // The width of the field in inches
  public static final double fieldWidth = Units.inchesToMeters(317);

  // The center of the field as a Translation2d
  public static final Translation2d fieldCenter =
      new Translation2d(fieldLength / 2, fieldWidth / 2);

  // The distance to the starting line from the driver station measured from the inside of the
  // starting zone
  public static final double startingLineX = Units.inchesToMeters(299.438);

  /** Processor location and size constants */
  public static class Processor {
    public static final List<Pose2d> centerFaces = new ArrayList<>();

    // The width of the processor in inches
    public static final double processorWidth = Units.inchesToMeters(46.25);

    static {
      centerFaces.add(
          new Pose2d(
              aprilTagLayout.getTagPose(3).get().toPose2d().getTranslation(),
              aprilTagLayout
                  .getTagPose(3)
                  .get()
                  .toPose2d()
                  .getRotation()
                  .rotateBy(Rotation2d.k180deg)));
      centerFaces.add(
          new Pose2d(
              aprilTagLayout.getTagPose(16).get().toPose2d().getTranslation(),
              aprilTagLayout
                  .getTagPose(16)
                  .get()
                  .toPose2d()
                  .getRotation()
                  .rotateBy(Rotation2d.k180deg)));
    }
  }

  /** Barge location and size constants */
  public static class Barge {
    // The width of the net in inches
    public static final double netWidth = Units.inchesToMeters(40.0);

    // The height of the net (measured at the sides) in inches
    public static final double netHeight = Units.inchesToMeters(88.0);

    // The distance from the driver station to the center of the cages in inches. The Y distance is
    // always measured off the blue alliance wall.
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  /** Coral Station location and size constants */
  public static class CoralStation {
    // The width of the station in inches
    public static final double stationWidth = Units.inchesToMeters(79.750);

    public static final List<Pose2d> centerFaces = new ArrayList<>();

    static {
      centerFaces.add(
          new Pose2d(
              aprilTagLayout.getTagPose(1).get().toPose2d().getTranslation(),
              aprilTagLayout
                  .getTagPose(1)
                  .get()
                  .toPose2d()
                  .getRotation()
                  .rotateBy(Rotation2d.k180deg)));
      centerFaces.add(
          new Pose2d(
              aprilTagLayout.getTagPose(2).get().toPose2d().getTranslation(),
              aprilTagLayout
                  .getTagPose(2)
                  .get()
                  .toPose2d()
                  .getRotation()
                  .rotateBy(Rotation2d.k180deg)));
      centerFaces.add(
          new Pose2d(
              aprilTagLayout.getTagPose(12).get().toPose2d().getTranslation(),
              aprilTagLayout
                  .getTagPose(12)
                  .get()
                  .toPose2d()
                  .getRotation()
                  .rotateBy(Rotation2d.k180deg)));
      centerFaces.add(
          new Pose2d(
              aprilTagLayout.getTagPose(13).get().toPose2d().getTranslation(),
              aprilTagLayout
                  .getTagPose(13)
                  .get()
                  .toPose2d()
                  .getRotation()
                  .rotateBy(Rotation2d.k180deg)));
    }
  }

  /** A simple enumerator for defining the two sides of a reef */
  public enum ReefSide {
    LEFT,
    RIGHT
  }

  /** Reef location and size constants */
  public static class Reef {
    // Length of the face of a reef side in inches
    public static final double faceLength = Units.inchesToMeters(36.792600);

    // Center of the reef measured from the driver station
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), fieldWidth / 2.0);

    // Side of the reef to the inside of the reef zone line
    public static final double faceToZoneLine = Units.inchesToMeters(12);

    // Starting facing the driver station in clockwise order
    public static final List<Pose2d> centerFaces = new ArrayList<>();

    public static final Pose2d[] redFaces = new Pose2d[6];
    public static final Pose2d[] blueFaces = new Pose2d[6];

    // Starting at the right branch facing the driver station in clockwise
    public static final List<Pose2d> branchPositions = new ArrayList<>();

    static {
      blueFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
      blueFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
      blueFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
      blueFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
      blueFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
      blueFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d();

      redFaces[0] = aprilTagLayout.getTagPose(7).get().toPose2d();
      redFaces[1] = aprilTagLayout.getTagPose(6).get().toPose2d();
      redFaces[2] = aprilTagLayout.getTagPose(11).get().toPose2d();
      redFaces[3] = aprilTagLayout.getTagPose(10).get().toPose2d();
      redFaces[4] = aprilTagLayout.getTagPose(9).get().toPose2d();
      redFaces[5] = aprilTagLayout.getTagPose(8).get().toPose2d();

      // Do the red faces
      for (int face = 0; face < 6; face++) {
        Pose2d poseDirection =
            new Pose2d(
                redFaces[face].getTranslation(),
                Rotation2d.fromDegrees(redFaces[face].getRotation().getDegrees() - 180));
        double adjustX = Units.inchesToMeters(30.738);

        var rightBranchPose =
            new Pose2d(
                new Translation2d(
                    poseDirection
                        .transformBy(
                            new Transform2d(adjustX, Units.inchesToMeters(5.1), new Rotation2d()))
                        .getX(),
                    poseDirection
                        .transformBy(
                            new Transform2d(adjustX, Units.inchesToMeters(5.1), new Rotation2d()))
                        .getY()),
                new Rotation2d(poseDirection.getRotation().getRadians()));

        var leftBranchPose =
            new Pose2d(
                new Translation2d(
                    poseDirection
                        .transformBy(
                            new Transform2d(adjustX, Units.inchesToMeters(-6.75), new Rotation2d()))
                        .getX(),
                    poseDirection
                        .transformBy(
                            new Transform2d(adjustX, Units.inchesToMeters(-6.75), new Rotation2d()))
                        .getY()),
                new Rotation2d(poseDirection.getRotation().getRadians()));

        branchPositions.add(rightBranchPose);
        branchPositions.add(leftBranchPose);
        centerFaces.add(redFaces[face]);
      }

      // Do the blue faces
      for (int face = 0; face < 6; face++) {
        Pose2d poseDirection =
            new Pose2d(
                blueFaces[face].getTranslation(),
                Rotation2d.fromDegrees(redFaces[face].getRotation().getDegrees() - 180));
        double adjustX = Units.inchesToMeters(30.738);

        var rightBranchPose =
            new Pose2d(
                new Translation2d(
                    poseDirection
                        .transformBy(
                            new Transform2d(adjustX, Units.inchesToMeters(5.1), new Rotation2d()))
                        .getX(),
                    poseDirection
                        .transformBy(
                            new Transform2d(adjustX, Units.inchesToMeters(5.1), new Rotation2d()))
                        .getY()),
                new Rotation2d(poseDirection.getRotation().getRadians()));

        var leftBranchPose =
            new Pose2d(
                new Translation2d(
                    poseDirection
                        .transformBy(
                            new Transform2d(adjustX, Units.inchesToMeters(-6.75), new Rotation2d()))
                        .getX(),
                    poseDirection
                        .transformBy(
                            new Transform2d(adjustX, Units.inchesToMeters(-6.75), new Rotation2d()))
                        .getY()),
                new Rotation2d(poseDirection.getRotation().getRadians()));

        branchPositions.add(rightBranchPose);
        branchPositions.add(leftBranchPose);
        centerFaces.add(blueFaces[face]);
      }

      // Initialize blue branch positions
      // for (int face = 0; face < 6; face++) {
      //   Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
      //   double adjustX = Units.inchesToMeters(30.738);

      //   var rightBranchPose =
      //       new Pose2d(
      //           new Translation2d(
      //               poseDirection
      //                   .transformBy(
      //                       new Transform2d(adjustX, Units.inchesToMeters(5.1), new
      // Rotation2d()))
      //                   .getX(),
      //               poseDirection
      //                   .transformBy(
      //                       new Transform2d(adjustX, Units.inchesToMeters(5.1), new
      // Rotation2d()))
      //                   .getY()),
      //           new Rotation2d(poseDirection.getRotation().getRadians()));

      //   var leftBranchPose =
      //       new Pose2d(
      //           new Translation2d(
      //               poseDirection
      //                   .transformBy(
      //                       new Transform2d(adjustX, Units.inchesToMeters(-6.75), new
      // Rotation2d()))
      //                   .getX(),
      //               poseDirection
      //                   .transformBy(
      //                       new Transform2d(adjustX, Units.inchesToMeters(-6.75), new
      // Rotation2d()))
      //                   .getY()),
      //           new Rotation2d(poseDirection.getRotation().getRadians()));

      //   branchPositions.add(rightBranchPose);
      //   branchPositions.add(leftBranchPose);
      // }
    }
  }

  /**
   * Get the position of center of the nearest reef face.
   *
   * @param currentPose The robots current position on the field
   * @return The position of the nearest reef face
   */
  public static Pose2d getNearestReefFace(Pose2d currentPose) {
    return currentPose.nearest(FieldConstants.Reef.centerFaces);
  }

  /**
   * Get the position of the nearest reef branch. This will find the nearest reef face, and then
   * depending on if you want the left or right branch it will then return the offset position for
   * that branch of the reef.
   *
   * @param currentPose The robots current position on the field
   * @param side The side of the reef you want to get the position to
   * @return The position of the nearest requested branch of the reef
   */
  public static Pose2d getNearestReefBranch(Pose2d currentPose, ReefSide side) {
    return FieldConstants.Reef.branchPositions.get(
        FieldConstants.Reef.centerFaces.indexOf(getNearestReefFace(currentPose)) * 2
            + (side == ReefSide.LEFT ? 1 : 0));
  }

  /**
   * Get the position of the nearest coral station.
   *
   * @param currentPose The robots current location
   * @return The position of the nearest coral station to the robot
   */
  public static Pose2d getNearestCoralStation(Pose2d currentPose) {
    return currentPose.nearest(FieldConstants.CoralStation.centerFaces);
  }

  /**
   * Get the position of the nearest processor
   *
   * @param currentPose The robots current location
   * @return The position of the nearest processor to the robot
   */
  public static Pose2d getNearestProcessorFace(Pose2d currentPose) {
    return currentPose.nearest(FieldConstants.Processor.centerFaces);
  }
}
