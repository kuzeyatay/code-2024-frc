package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  // The arm gearbox represents a gearbox containing one neo motor.

  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_arm_Sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(2),
          6,
          0.0005,
          0.254,
          Units.degreesToRadians(-100),
          Units.degreesToRadians(360),
          false,
          -20);
  private double armTopAppliedVolts = 0.0;

  public ArmIOSim() {
    m_arm_Sim.setState(VecBuilder.fill(Math.PI / 2.0, 0.0));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    m_arm_Sim.update(0.002);

    inputs.armInternalPositionRad = m_arm_Sim.getAngleRads();

    inputs.armInternalVelocityRadPerSec = m_arm_Sim.getVelocityRadPerSec();
    inputs.armAppliedVolts = armTopAppliedVolts;
    inputs.armCurrentAmps = new double[] {m_arm_Sim.getCurrentDrawAmps()};
    inputs.armTempCelcius = new double[] {};
  }

  @Override
  public void setArmVoltage(double voltage) {
    armTopAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    m_arm_Sim.setInputVoltage(armTopAppliedVolts);
  }
}
