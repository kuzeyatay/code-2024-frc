package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.Leds;

public class feedAndShoot extends ParallelCommandGroup {
  /** Creates a new feedAndShoot. */
  public feedAndShoot(Intake intake, Shooter shooter, Leds leds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        intake.rollerForSecs(),
        shooter.launchAutoCommand(),
        new InstantCommand(() -> leds.hpThrowGamePiece = true));
  }
}
