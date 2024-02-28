package frc.robot.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionConstants {
  // Measurements in METERS

  public static double AprilTag3Height =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(3).get().getY();

  public static double AprilTag4Height =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(4).get().getY();

  public static double AprilTag7Height =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(7).get().getY();

  public static double AprilTag8Height =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(8).get().getY();

  // Cam mounted facing forward, half a meter backwards of center, a meter up from center for
  // photonvision
  public static Transform3d kRobotToCam1 =
      new Transform3d(
          new Translation3d(-0.5, 0.0, 0.5), new Rotation3d(0, Units.degreesToRadians(0), 0));
  public static Transform3d kRobotToCam2 =
      new Transform3d(
          new Translation3d(-0.5, 0.0, 0.5), new Rotation3d(0, Units.degreesToRadians(0), Math.PI));

  // The layout of the AprilTags on the field(season specific)
  public static AprilTagFieldLayout kTagLayout =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  public static Pose3d AprilTag1 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(1).get();

  public static Pose3d AprilTag2 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(2).get();

  public static Pose3d AprilTag3 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(3).get();

  public static Pose3d AprilTag4 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(4).get();

  public static Pose3d AprilTag5 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(5).get();

  public static Pose3d AprilTag6 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(6).get();

  public static Pose3d AprilTag7 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(1).get();

  public static Pose3d AprilTag8 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(8).get();

  public static Pose3d AprilTag9 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(9).get();

  public static Pose3d AprilTag10 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(10).get();

  public static Pose3d AprilTag11 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(11).get();

  public static Pose3d AprilTag12 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(12).get();

  public static Pose3d AprilTag13 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(13).get();

  public static Pose3d AprilTag14 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(14).get();

  public static Pose3d AprilTag15 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(15).get();

  public static Pose3d AprilTag16 =
      AprilTagFields.kDefaultField.loadAprilTagLayoutField().getTagPose(16).get();

  public static final class Coordinates {
    public static final Pose2d BLUE_SPEAKER = new Pose2d(0.22, 5.55, new Rotation2d(Math.PI));
    public static final Pose2d BLUE_AMP = new Pose2d(1.79, 7.60, new Rotation2d(Math.PI / 2));
    public static final Pose2d BLUE_SOURCE = new Pose2d(15.3, 1.11, Rotation2d.fromDegrees(-55));
    public static final Pose2d RED_SPEAKER =
        new Pose2d(16.54175 - 0.22, 5.55, new Rotation2d(Math.PI));
    public static final Pose2d RED_AMP = new Pose2d(14.68, 7.52, new Rotation2d(Math.PI / 2));
    public static final Pose2d RED_SOURCE = new Pose2d(1.14, 1.00, Rotation2d.fromDegrees(-120));
    public static final Pose2d RED_STAGE = new Pose2d(13, 2, Rotation2d.fromDegrees(120));
  }

  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  public static Boolean cestRed() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return true;
    } else {
      return false;
    }
  }

  public static Pose2d getAllianceSpeaker() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return Coordinates.RED_SPEAKER;
    } else {
      return Coordinates.BLUE_SPEAKER;
    }
  }
  /** Each corner of the speaker * */
  public static final class Speaker {

    // corners (blue alliance origin)
    public static Translation3d topRightSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(238.815),
            Units.inchesToMeters(83.091));

    public static Translation3d topLeftSpeaker =
        new Translation3d(
            Units.inchesToMeters(18.055),
            Units.inchesToMeters(197.765),
            Units.inchesToMeters(83.091));

    public static Translation3d bottomRightSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
    public static Translation3d bottomLeftSpeaker =
        new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

    /** Center of the speaker opening (blue alliance) */
    public static Translation3d centerSpeakerOpening =
        bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5);
  }

  public static double aprilTagWidth = Units.inchesToMeters(6.50);
  public static AprilTagFieldLayout aprilTags;
}
