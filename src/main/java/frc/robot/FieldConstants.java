package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Angle;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  // Set the April Tag field to use
  public static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  private static final Angle rotationOffset = Degree.of(-5);
  private static final double faceOffset = Meters.convertFrom(18.5, Inches);

  public enum ReefAlignLocation {
    LEFT,
    RIGHT,
    CENTER
  }

  public static class SourceAlign {
    public static final List<Pose2d> centerPositions = new ArrayList<>();

    static {
      centerPositions.add(
          aprilTagLayout
              .getTagPose(1)
              .get()
              .toPose2d()
              .transformBy(
                  new Transform2d(
                      (faceOffset - Meters.convertFrom(0.5, Inches)),
                      0,
                      new Rotation2d(rotationOffset))));
      centerPositions.add(
          aprilTagLayout
              .getTagPose(2)
              .get()
              .toPose2d()
              .transformBy(
                  new Transform2d(
                      (faceOffset - Meters.convertFrom(0.5, Inches)),
                      0,
                      new Rotation2d(rotationOffset))));
      centerPositions.add(
          aprilTagLayout
              .getTagPose(12)
              .get()
              .toPose2d()
              .transformBy(
                  new Transform2d(
                      (faceOffset - Meters.convertFrom(0.5, Inches)),
                      0,
                      new Rotation2d(rotationOffset))));
      centerPositions.add(
          aprilTagLayout
              .getTagPose(13)
              .get()
              .toPose2d()
              .transformBy(
                  new Transform2d(
                      (faceOffset - Meters.convertFrom(0.5, Inches)),
                      0,
                      new Rotation2d(rotationOffset))));
    }
  }

  public static class ProcessorAlign {
    public static final List<Pose2d> centerPositions = new ArrayList<>();

    static {
      centerPositions.add(
          aprilTagLayout
              .getTagPose(3)
              .get()
              .toPose2d()
              .transformBy(
                  new Transform2d(
                      faceOffset, 0, Rotation2d.k180deg.plus(new Rotation2d(rotationOffset)))));
      centerPositions.add(
          aprilTagLayout
              .getTagPose(16)
              .get()
              .toPose2d()
              .transformBy(
                  new Transform2d(
                      faceOffset, 0, Rotation2d.k180deg.plus(new Rotation2d(rotationOffset)))));
    }
  }

  public static class ReefAlign {
    public static final List<Pose2d> centerPositions = new ArrayList<>();
    public static final List<Pose2d> leftBranchPositions = new ArrayList<>();
    public static final List<Pose2d> rightBranchPositions = new ArrayList<>();

    private static final Pose2d[] blueTags = new Pose2d[6];
    private static final Pose2d[] redTags = new Pose2d[6];

    private static final double leftBranchOffset = Meters.convertFrom(-6.5, Inches);
    private static final double rightBranchOffset = Meters.convertFrom(6.5, Inches);

    static {
      blueTags[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
      blueTags[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
      blueTags[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
      blueTags[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
      blueTags[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
      blueTags[5] = aprilTagLayout.getTagPose(17).get().toPose2d();

      for (int face = 0; face < 6; face++) {
        var centerPos =
            blueTags[face].transformBy(
                new Transform2d(
                    faceOffset, 0, Rotation2d.k180deg.plus(new Rotation2d(rotationOffset))));
        var leftBranchPos =
            blueTags[face].transformBy(
                new Transform2d(
                    (faceOffset - Meters.convertFrom(0.5, Inches)),
                    leftBranchOffset,
                    Rotation2d.k180deg.plus(new Rotation2d(rotationOffset))));
        var rightBranchPos =
            blueTags[face].transformBy(
                new Transform2d(
                    faceOffset,
                    rightBranchOffset,
                    Rotation2d.k180deg.plus(new Rotation2d(rotationOffset))));

        centerPositions.add(centerPos);
        leftBranchPositions.add(leftBranchPos);
        rightBranchPositions.add(rightBranchPos);

        // System.out.println("Blue Face " + (face + 1) + ":");
        // System.out.println("  Center:");
        // System.out.println("    X: " + centerPos.getMeasureX());
        // System.out.println("    Y: " + centerPos.getMeasureY());
        // System.out.println("    0: " + centerPos.getRotation().getDegrees());
        // System.out.println("  Left:");
        // System.out.println("    X: " + leftBranchPos.getMeasureX());
        // System.out.println("    Y: " + leftBranchPos.getMeasureY());
        // System.out.println("    0: " + leftBranchPos.getRotation().getDegrees());
        // System.out.println("  Right:");
        // System.out.println("    X: " + rightBranchPos.getMeasureX());
        // System.out.println("    Y: " + rightBranchPos.getMeasureY());
        // System.out.println("    0: " + rightBranchPos.getRotation().getDegrees());
        // System.out.println("");
      }

      redTags[0] = aprilTagLayout.getTagPose(7).get().toPose2d();
      redTags[1] = aprilTagLayout.getTagPose(6).get().toPose2d();
      redTags[2] = aprilTagLayout.getTagPose(11).get().toPose2d();
      redTags[3] = aprilTagLayout.getTagPose(10).get().toPose2d();
      redTags[4] = aprilTagLayout.getTagPose(9).get().toPose2d();
      redTags[5] = aprilTagLayout.getTagPose(8).get().toPose2d();

      for (int face = 0; face < 6; face++) {
        var centerPos =
            redTags[face].transformBy(
                new Transform2d(
                    faceOffset, 0, Rotation2d.k180deg.plus(new Rotation2d(rotationOffset))));
        var leftBranchPos =
            redTags[face].transformBy(
                new Transform2d(
                    (faceOffset - Meters.convertFrom(0.5, Inches)),
                    leftBranchOffset,
                    Rotation2d.k180deg.plus(new Rotation2d(rotationOffset))));
        var rightBranchPos =
            redTags[face].transformBy(
                new Transform2d(
                    faceOffset,
                    rightBranchOffset,
                    Rotation2d.k180deg.plus(new Rotation2d(rotationOffset))));

        centerPositions.add(centerPos);
        leftBranchPositions.add(leftBranchPos);
        rightBranchPositions.add(rightBranchPos);

        // System.out.println("Red Face " + (face + 1) + ":");
        // System.out.println("  Center:");
        // System.out.println("    X: " + centerPos.getMeasureX());
        // System.out.println("    Y: " + centerPos.getMeasureY());
        // System.out.println("    0: " + centerPos.getRotation().getDegrees());
        // System.out.println("  Left:");
        // System.out.println("    X: " + leftBranchPos.getMeasureX());
        // System.out.println("    Y: " + leftBranchPos.getMeasureY());
        // System.out.println("    0: " + leftBranchPos.getRotation().getDegrees());
        // System.out.println("  Right:");
        // System.out.println("    X: " + rightBranchPos.getMeasureX());
        // System.out.println("    Y: " + rightBranchPos.getMeasureY());
        // System.out.println("    0: " + rightBranchPos.getRotation().getDegrees());
        // System.out.println("");
      }

      Logger.recordOutput("Tags/Center Positions", centerPositions.toArray(new Pose2d[12]));
      Logger.recordOutput(
          "Tags/Left Branchs Positions", leftBranchPositions.toArray(new Pose2d[12]));
      Logger.recordOutput(
          "Tags/Right Branch Positions", rightBranchPositions.toArray(new Pose2d[12]));
    }
  }

  public static Pose2d getNearestSource(Pose2d currentPose) {
    return currentPose.nearest(SourceAlign.centerPositions);
  }

  public static Pose2d getNearestProcessor(Pose2d currentPose) {
    return currentPose.nearest(ProcessorAlign.centerPositions);
  }

  public static Pose2d getNearestReefFace(Pose2d currentPose) {
    return currentPose.nearest(ReefAlign.centerPositions);
  }

  public static Pose2d getNearestLeftBranch(Pose2d currentPose) {
    return currentPose.nearest(ReefAlign.leftBranchPositions);
  }

  public static Pose2d getNearestRightBranch(Pose2d currentPose) {
    return currentPose.nearest(ReefAlign.rightBranchPositions);
  }
}
