package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.Leds;

public class feedAndStop extends ParallelCommandGroup {
  /** Creates a new feedAndShoot. */
  public feedAndStop(Intake intake, Shooter shooter, Leds leds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        intake.rollerForSecs(),
        shooter.intakeCommand(),
        new InstantCommand(() -> leds.hpThrowGamePiece = true));
  }
}
