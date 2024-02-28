package frc.robot.util.SparkMax;

import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

/** Preset configurations for Spark Max periodic frame rates. */
public class SparkMaxPeriodicFrameConfig {
  public static void configNotLeader(CANSparkMax sparkMax) {
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
  }

  public static void configLeaderFollower(CANSparkMax sparkMax) {
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
  }
}
