package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
  // tuned it to perfection manually dont touch
  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          6,
          0.001,
          0.254,
          Units.degreesToRadians(-100),
          Units.degreesToRadians(360),
          false,
          -30);
  private double armAppliedVolts = 0.0;

  private DCMotorSim rollerSim = new DCMotorSim(DCMotor.getCIM(1), 3, 0.0001);
  private double rollerAppliedVolts = 0.0;

  public IntakeIOSim() {
    armSim.setState(VecBuilder.fill(Math.PI / 2.0, 0.0));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    armSim.update(0.002);

    inputs.armInternalPositionRad = armSim.getAngleRads();
    inputs.armInternalVelocityRadPerSec = armSim.getVelocityRadPerSec();
    inputs.armAppliedVolts = armAppliedVolts;
    inputs.armCurrentAmps = new double[] {armSim.getCurrentDrawAmps()};
    inputs.armTempCelcius = new double[] {};

    inputs.rollerAppliedVolts = rollerAppliedVolts;
    inputs.rollerCurrentAmps = new double[] {rollerSim.getCurrentDrawAmps()};
  }

  @Override
  public void setArmVoltage(double voltage) {
    armAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    armSim.setInputVoltage(armAppliedVolts);
  }

  @Override
  public void setRollerVoltage(double voltage) {
    rollerAppliedVolts = MathUtil.clamp(voltage, -12.0, 12.0);
    rollerSim.setInputVoltage(rollerAppliedVolts);
  }
}
