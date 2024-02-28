package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Vision.VisionConstants;
import frc.robot.commands.arm.armCommands;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.FeedForwardCharacterization;
import frc.robot.commands.shooter.feedAndShoot;
import frc.robot.commands.shooter.feedAndStop;
import frc.robot.commands.shooter.feedAndStopTeleop;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Shooter.ShooterIOSparkMax;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavx;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.util.Leds;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Shuffleboard
  // private final ShuffleboardTab cameraTab = Shuffleboard.getTab("Camera");

  // Subsystems
  private final Drive drive;
  private final Shooter shooter;
  private final Intake intake;
  private final Arm arm;
  // private final Climber climber;
  private Leds leds = Leds.getInstance();
  // private final VisionSystem visionSystem = new VisionSystem(cameraTab);

  // Controller
  private final CommandPS4Controller driver = new CommandPS4Controller(0);
  private final CommandPS5Controller operator = new CommandPS5Controller(0);

  // Dashboard inputs
  private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    switch (ModeSet.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        drive =
            new Drive(
                new GyroIONavx(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));

        shooter = new Shooter(new ShooterIOSparkMax());
        intake = new Intake(new IntakeIOSparkMax());
        arm = new Arm(new ArmIOSparkMax());
        // climber = new Climber(new ClimberIOSparkMax());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        shooter = new Shooter(new ShooterIOSim());
        intake = new Intake(new IntakeIOSim());
        arm = new Arm(new ArmIOSim());
        // climber = new Climber(new ClimberIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations

        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        shooter = new Shooter(new ShooterIO() {});
        intake = new Intake(new IntakeIO() {});
        arm = new Arm(new ArmIO() {});
        // climber = new Climber(new ClimberIO() {});

        break;
    }

    // Smart Dashboard
    // SmartDashboard.putData(visionSystem);

    autoChooser = AutoBuilder.buildAutoChooser();

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));

    // registering commands for pathplanner
    NamedCommands.registerCommand("shootAtStart", new feedAndShoot(intake, shooter, leds));
    NamedCommands.registerCommand("inAndOut", intake.runAutoOutCommand());
    NamedCommands.registerCommand("shooterFeed", new feedAndStop(intake, shooter, leds));
    NamedCommands.registerCommand("armOpen", arm.autoAmp());
    NamedCommands.registerCommand("armIn", arm.Neutral());
    NamedCommands.registerCommand("shooterAtPos", arm.autoIntake());
    NamedCommands.registerCommand("distanceShoot", arm.autoShootFromDistance());
    // Configure the button bindings
    configureButtonBindings();

    // initiate pathplanner
    Pathplanner();

    // Endgame Alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(20.0)) // 20 secs left
        .onTrue(
            Commands.run(
                    () -> {
                      Leds.getInstance().endgameAlert = true;
                    })
                .withTimeout(1.5)
                .andThen(
                    Commands.run(
                            () -> {
                              Leds.getInstance().endgameAlert = false;
                              driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                              operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                            })
                        .withTimeout(1.0)));

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(10.0)) // 10 secs left
        .onTrue(
            Commands.sequence(
                Commands.run(
                        () -> {
                          Leds.getInstance().endgameAlert = true;
                          driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                          operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                        })
                    .withTimeout(0.5),
                Commands.run(
                        () -> {
                          Leds.getInstance().endgameAlert = false;
                          driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                          operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                        })
                    .withTimeout(0.5),
                Commands.run(
                        () -> {
                          Leds.getInstance().endgameAlert = true;
                          driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                          operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                        })
                    .withTimeout(0.5),
                Commands.run(
                        () -> {
                          Leds.getInstance().endgameAlert = false;
                          driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                          operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                        })
                    .withTimeout(1.0)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Chassis commands
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -operator.getLeftY(),
            () -> -operator.getLeftX(),
            () -> -operator.getRawAxis(2)
            // () -> -controller.getRawAxis(2)
            ));

    // controller.y().onTrue(Commands.runOnce(drive::stopWithX, drive));

    driver
        .circle()
        .whileTrue(
            DriveCommands.alignWhileDrivingCommand(
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> VisionConstants.getAllianceSpeaker().getTranslation(),
                drive));

    // controller.povCenter().onTrue(Commands.runOnce(drive::setEstimPoseToOdometry, drive));
    // controller.b().onTrue(Commands.runOnce(() -> opt.setTargetMode()));

    // shooter commands
    operator.L3().whileTrue(shooter.intakeCommand());
    operator.R3().whileTrue(shooter.launchCommand());
    operator.povDown().whileTrue(shooter.feederPushCommand());
    operator.povUp().whileTrue(shooter.launchAutoCommand());

    operator
        .PS()
        .onTrue(Commands.run(() -> operator.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

    // intake commands
    operator.L1().onTrue(new feedAndStopTeleop(intake, shooter));
    operator.R1().onTrue(intake.runInCommand());

    // arm commands
    operator.PS().onTrue(arm.Neutral());
    operator.circle().whileTrue(armCommands.alignWhileDrivingCommand(drive, arm));

    operator.cross().onTrue(arm.deliverAmp());
    operator.triangle().onTrue(arm.shootFromDistance());
    operator.square().onTrue(arm.intakeChute());

    // climber commands
    // operator.povUp().onTrue(climber.extend());
    // operator.povDown().onTrue(climber.extend());
    // controller.b().onTrue(Commands.runOnce(() ->drive.setPose( new
    // Pose2d(drive.getPose().getTranslation(), new Rotation2d())),drive).ignoringDisable(true));
  }

  private void Pathplanner() {

    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser
    // built above
    SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));
    SmartDashboard.putData("5 Note Auto", new PathPlannerAuto("5 Note Auto"));
    SmartDashboard.putData("3+1 Note Auto", new PathPlannerAuto("3+1 Note Auto"));

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m forward of its current position
    SmartDashboard.putData(
        "custom trajectory",
        Commands.runOnce(
            () -> {
              Pose2d currentPose = drive.getPose();

              // The rotation component in these poses represents the direction of travel
              Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
              Pose2d endPos =
                  new Pose2d(new Translation2d(1.823, 0.672), Rotation2d.fromRotations(0));

              List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
              PathPlannerPath path =
                  new PathPlannerPath(
                      bezierPoints,
                      new PathConstraints(
                          4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
                      new GoalEndState(0.0, Rotation2d.fromRotations(0)));

              // Prevent this path from being flipped on the red alliance, since the given positions
              // are already correct
              path.preventFlipping = true;
              AutoBuilder.followPath(path).schedule();
            }));

    // Add a button to run pathfinding commands to SmartDashboard
    SmartDashboard.putData(
        "Pathfind to Pickup Pos",
        AutoBuilder.pathfindToPose(
            new Pose2d(14.77, 1.25, Rotation2d.fromDegrees(-140)),
            new PathConstraints(4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
            0,
            2.0));

    SmartDashboard.putData(
        "Pathfind to Scoring Pos",
        AutoBuilder.pathfindToPose(
            new Pose2d(2.15, 3.0, Rotation2d.fromDegrees(180)),
            new PathConstraints(4.0, 4.0, Units.degreesToRadians(360), Units.degreesToRadians(540)),
            0,
            0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return Commands.sequence(new followTrajectory(drive));
  }
}
