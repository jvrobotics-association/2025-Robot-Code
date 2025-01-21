// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.BlinkinLEDController.BlinkinPattern;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SetpointConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ExtendClimberCommand;
import frc.robot.commands.RetractClimberCommand;
import frc.robot.commands.RunSmartIntake;
import frc.robot.commands.RunSmartOuttake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private static final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private static final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  // Controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandJoystick m_operatorPanel =
      new CommandJoystick(OperatorConstants.kOperatorPanelPort);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
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
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // #########################
    // ###  INTAKE COMMANDS  ###
    // #########################

    // Run the intake until a note is loaded and ensure it is loaded ("Intake In")
    m_driverController
        .leftTrigger()
        .onTrue(
            new SequentialCommandGroup(
                new RunSmartIntake(m_intakeSubsystem),
                Commands.waitSeconds(0.2),
                new RunSmartOuttake(m_intakeSubsystem),
                Commands.runOnce(
                    () ->
                        BlinkinLEDController.getInstance().setPattern(BlinkinPattern.STROBE_WHITE)),
                Commands.waitSeconds(1),
                Commands.runOnce(
                    () -> BlinkinLEDController.getInstance().setPattern(BlinkinPattern.GREEN))));

    // Run the intake in reverse ("Intake Out")
    m_operatorPanel
        .button(1)
        .whileTrue(
            Commands.startEnd(
                () -> m_intakeSubsystem.out(0.35),
                () -> {
                  m_intakeSubsystem.stopIntake();
                  checkNoteLightColor();
                },
                m_intakeSubsystem));

    // #######################
    // ###  ARM COMMANDS  ###
    // ######################

    // Pickup position
    m_operatorPanel
        .button(2)
        .onTrue(
            Commands.runOnce(
                () -> m_armSubsystem.setTargetPosition(ArmConstants.MINIMUM_ANGLE),
                m_armSubsystem));

    // Set the arm to the "Speaker Close" position ("Speaker Close")
    m_operatorPanel
        .button(3)
        .onTrue(
            Commands.runOnce(
                () -> m_armSubsystem.setTargetPosition(SetpointConstants.closeSpeakerArmAngle),
                m_armSubsystem));

    // Set the arm to the "Speaker Far" position ("Speaker Far")
    m_operatorPanel
        .button(4)
        .onTrue(
            Commands.runOnce(
                () -> m_armSubsystem.setTargetPosition(SetpointConstants.farSpeakerArmAngle),
                m_armSubsystem));

    // Raise the arm to the back position for scoring in the amp ("Amp")
    m_operatorPanel
        .button(6)
        .onTrue(
            new SequentialCommandGroup(
                Commands.runOnce(() -> m_armSubsystem.raiseArmOpenLoop(0.5), m_armSubsystem),
                Commands.waitSeconds(1.5),
                Commands.runOnce(() -> m_armSubsystem.stop(), m_armSubsystem)));

    // Manually raise the arm ("Arm Up")
    m_operatorPanel
        .button(12)
        .whileTrue(
            Commands.runEnd(
                () -> m_armSubsystem.raiseArm(),
                () -> {
                  if (m_armSubsystem.getDegrees() >= ArmConstants.POSITION_HOLDING_CUTOFF) {
                    m_armSubsystem.stop();
                  }
                },
                m_armSubsystem));

    // Manually lower the arm ("Arm Down")
    m_operatorPanel
        .button(11)
        .whileTrue(
            Commands.runEnd(
                () -> m_armSubsystem.lowerArm(),
                () -> {
                  if (m_armSubsystem.getDegrees() >= ArmConstants.POSITION_HOLDING_CUTOFF) {
                    m_armSubsystem.stop();
                  }
                },
                m_armSubsystem));

    // ###########################
    // ###  Shooter COMMANDS  ###
    // ##########################

    // Shoot - this command adjusts the shooter speed based on the postiion of the arm
    m_operatorPanel
        .button(5)
        .onTrue(
            new SequentialCommandGroup(
                // Run the shooter at different speeds based on the arm angle
                Commands.runOnce(
                    () -> {
                      if (m_armSubsystem.getDegrees() > 85) {
                        m_shooterSubsystem.shoot(SetpointConstants.ampShooterSpeed);
                      } else if (m_armSubsystem.getTargetPositionInDegreesNullSafe()
                          == SetpointConstants.farSpeakerArmAngle) {
                        m_shooterSubsystem.shoot(SetpointConstants.farSpeakerShooterSpeed);
                      } else if (m_armSubsystem.getTargetPositionInDegreesNullSafe()
                          == SetpointConstants.closeSpeakerArmAngle) {
                        m_shooterSubsystem.shoot(SetpointConstants.closeSpeakerShooterSpeed);
                      } else m_shooterSubsystem.shoot(0.5);
                    },
                    m_shooterSubsystem),

                // Wait 1 second for the shooter to get to the proper speed
                Commands.waitSeconds(1),

                // Run the intake so that it feeds the note into the shooter
                Commands.runOnce(() -> m_intakeSubsystem.in(1), m_intakeSubsystem),

                // Wait so that the note is out of the shooter before stopping the intake
                Commands.waitSeconds(0.5),

                // Stop the shooter and return the arm to the pickup position
                new ParallelCommandGroup(
                    Commands.runOnce(
                        () -> m_armSubsystem.setTargetPosition(ArmConstants.MINIMUM_ANGLE),
                        m_armSubsystem),
                    Commands.runOnce(
                        () -> {
                          m_shooterSubsystem.stopShooter();
                          m_intakeSubsystem.stopIntake();
                          checkNoteLightColor();
                        },
                        m_shooterSubsystem,
                        m_intakeSubsystem))));

    // ########################
    // ### Climber commands ###
    // ########################

    // Extend the climbing mechanism
    m_operatorPanel
        .button(10)
        .onTrue(
            new SequentialCommandGroup(
                Commands.runOnce(() -> m_climberSubsystem.unlockBreak(), m_climberSubsystem),
                Commands.waitSeconds(0.2),
                new ExtendClimberCommand(m_climberSubsystem)));

    // Retract the climber mechanism
    m_operatorPanel
        .button(9)
        .onTrue(
            new SequentialCommandGroup(
                Commands.runOnce(() -> m_climberSubsystem.unlockBreak(), m_climberSubsystem),
                Commands.waitSeconds(0.2),
                new RetractClimberCommand(m_climberSubsystem),
                Commands.runOnce(() -> m_armSubsystem.setTargetPosition(53), m_armSubsystem)));

    // Manually retract the climber mechanism
    m_driverController
        .button(7)
        .whileTrue(
            new SequentialCommandGroup(
                Commands.runOnce(() -> m_climberSubsystem.unlockBreak(), m_climberSubsystem),
                Commands.waitSeconds(0.1),
                Commands.startEnd(
                    () -> m_climberSubsystem.manualRetract(),
                    () -> {
                      m_climberSubsystem.stopClimber();
                      m_climberSubsystem.zeroPosition();
                    },
                    m_climberSubsystem)));

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()));

    // Lock to 0° when A button is held
    m_driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -m_driverController.getLeftY(),
                () -> -m_driverController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    m_driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when start button is pressed
    m_driverController
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void checkNoteLightColor() {
    if (m_intakeSubsystem.noteIsLoaded()) {
      BlinkinLEDController.getInstance().setPattern(BlinkinPattern.GREEN);
    } else BlinkinLEDController.getInstance().setAllianceColorSolid();
  }
}
