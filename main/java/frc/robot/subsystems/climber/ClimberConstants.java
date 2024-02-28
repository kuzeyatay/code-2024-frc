package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class ClimberConstants {
  public static final double kSoftLimitReverse = 0.0;
  public static final double kSoftLimitForward = Units.degreesToRadians(360);

  public static final double kArmGearRatio = 0.108; // (24.0 / 54.0) * (1.0 / 36.0); // 0.0648;
  public static final double kPositionFactor =
      kArmGearRatio
          * 2.0
          * Math.PI; // multiply SM value by this number and get arm position in radians
  public static final double kVelocityFactor = kArmGearRatio * 2.0 * Math.PI / 60.0;
  public static final double kArmFreeSpeed = 5676.0 * kVelocityFactor;
  public static final double kArmZeroCosineOffset =
      0.0; // radians to add to converted arm position to get real-world arm position (starts at
  // ~90deg angle)
  public static final ArmFeedforward kArmFeedforward =
      new ArmFeedforward(0.0, 3.0, 12.0 / kArmFreeSpeed, 0.0);

  public static final TrapezoidProfile.Constraints kArmMotionConstraint =
      new TrapezoidProfile.Constraints(2.0, 2.0);
}
