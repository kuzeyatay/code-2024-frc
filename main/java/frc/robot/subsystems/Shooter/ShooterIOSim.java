package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private DCMotorSim topSim = new DCMotorSim(DCMotor.getCIM(1), 1, 0.0001);
  private DCMotorSim bottomSim = new DCMotorSim(DCMotor.getCIM(1), 1, 0.0001);
  private DCMotorSim feedSim = new DCMotorSim(DCMotor.getCIM(1), 1, 0.0001);

  private double topAppliedVolts = 0.0;
  private double bottomAppliedVolts = 0.0;
  private double feedAppliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    topSim.update(0.02);
    bottomSim.update(0.02);
    feedSim.update(0.02);

    inputs.topPositionRad = topSim.getAngularPositionRad();
    inputs.topVelocityRadPerSec = topSim.getAngularVelocityRadPerSec();
    inputs.topAppliedVolts = topAppliedVolts;
    inputs.topCurrentAmps = new double[] {topSim.getCurrentDrawAmps()};

    inputs.bottomPositionRad = bottomSim.getAngularPositionRad();
    inputs.bottomVelocityRadPerSec = bottomSim.getAngularVelocityRadPerSec();
    inputs.bottomAppliedVolts = bottomAppliedVolts;
    inputs.bottomCurrentAmps = new double[] {bottomSim.getCurrentDrawAmps()};

    inputs.feedPositionRad = feedSim.getAngularPositionRad();
    inputs.feedVelocityRadPerSec = feedSim.getAngularVelocityRadPerSec();
    inputs.feedAppliedVolts = feedAppliedVolts;
    inputs.feedCurrentAmps = feedSim.getCurrentDrawAmps();
  }

  @Override
  public void setTopVoltage(double volts) {
    topAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    topSim.setInputVoltage(topAppliedVolts);
  }

  @Override
  public void setBottomVoltage(double volts) {
    bottomAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    bottomSim.setInputVoltage(bottomAppliedVolts);
  }

  @Override
  public void setFeedVoltage(double volts) {
    feedAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    feedSim.setInputVoltage(feedAppliedVolts);
  }
}
