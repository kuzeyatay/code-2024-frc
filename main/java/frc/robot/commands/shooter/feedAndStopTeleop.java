package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.intake.Intake;

public class feedAndStopTeleop extends ParallelCommandGroup {
  /** Creates a new feedAndShoot. */
  public feedAndStopTeleop(Intake intake, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(intake.runOutCommand(), shooter.intakeCommand());
  }
}
