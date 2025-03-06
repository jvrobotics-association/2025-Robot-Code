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
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ReefLevel;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GamePieceCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralManipulator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
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
  private final Vision vision;
  private final Elevator elevator;
  private final CoralManipulator coralManipulator;
  private final Climber climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick operatorConsole = new CommandJoystick(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {

        // Initalize the
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.leftCameraName, VisionConstants.robotToLeftCamera),
                new VisionIOPhotonVision(
                    VisionConstants.rightCameraName, VisionConstants.robotToRightCamera));
        // Configure port forwarding for USB connections to the RoboRIO for PhotonVision
        PortForwarder.add(5800, "photonvision.local", 5800);
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

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.leftCameraName,
                    VisionConstants.robotToLeftCamera,
                    drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.rightCameraName,
                    VisionConstants.robotToRightCamera,
                    drive::getPose));
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
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up all other subsystems
    elevator = new Elevator();
    coralManipulator = new CoralManipulator();
    climber = new Climber();

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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Driver Right Bumper: Approach Nearest Right-Side Reef Branch
    controller
        .rightBumper()
        .whileTrue(
            DriveCommands.joystickApproach(
                drive,
                () -> -controller.getLeftY(),
                () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.RIGHT)));

    // Driver Left Bumper: Approach Nearest Left-Side Reef Branch
    controller
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickApproach(
                drive,
                () -> -controller.getLeftY(),
                () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.LEFT)));

    // Driver B button: Approach Nearest Reef Face (for removing algae)
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickApproach(
                drive,
                () -> -controller.getLeftY(),
                () -> FieldConstants.getNearestReefFace(drive.getPose())));

    // Lock to 0° when A button is held
    // controller
    //     .b()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when Y button is pressed
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Manually extend the climber
    controller
        .rightTrigger()
        .whileTrue(
            Commands.runEnd(() -> climber.manuallyExtend(), () -> climber.stopClimber(), climber));

    // Manually retract the climber
    controller
        .leftTrigger()
        .whileTrue(
            Commands.runEnd(() -> climber.manuallyRetract(), () -> climber.stopClimber(), climber));

    // Move the elevator to the L4 position
    operatorConsole
        .button(6)
        .onTrue(GamePieceCommands.placeCoralCommand(elevator, coralManipulator, ReefLevel.L4));

    // Move the elevator to the L3 coral position
    operatorConsole
        .button(3)
        .onTrue(GamePieceCommands.placeCoralCommand(elevator, coralManipulator, ReefLevel.L3));

    // Move the elevator to the L3 algae position
    operatorConsole
        .button(4)
        .onTrue(
            Commands.runOnce(() -> elevator.moveToPosition(ElevatorConstants.L3_ALGAE_POSITION)));

    // Move the elevator to the L2 coral position
    operatorConsole
        .button(5)
        .onTrue(GamePieceCommands.placeCoralCommand(elevator, coralManipulator, ReefLevel.L2));

    // Move the elevator to the L2 algae position
    operatorConsole
        .button(1)
        .onTrue(
            Commands.runOnce(() -> elevator.moveToPosition(ElevatorConstants.L2_ALGAE_POSITION)));

    // Move the elevator to the L1 algae position
    operatorConsole
        .button(10)
        .onTrue(GamePieceCommands.placeCoralCommand(elevator, coralManipulator, ReefLevel.L1));

    // Move the elevator to the algae processor scoring position
    operatorConsole
        .button(9)
        .onTrue(
            Commands.runOnce(
                () -> elevator.moveToPosition(ElevatorConstants.ALGAE_SCORE_POSITION)));

    // Manually raise the elevator without any PID control
    operatorConsole
        .button(12)
        .whileTrue(
            Commands.runEnd(
                () -> elevator.manuallyRaise(),
                () -> {
                  elevator.holdCurrentPosition();
                },
                elevator));

    // Manually lower the elevator without any PID control
    operatorConsole
        .button(11)
        .whileTrue(
            Commands.runEnd(
                () -> elevator.manuallyLower(),
                () -> {
                  elevator.holdCurrentPosition();
                },
                elevator));

    // Manually lower the elevator without any PID control
    operatorConsole
        .button(2)
        .whileTrue(Commands.run(() -> coralManipulator.setSpeed(0.3), coralManipulator));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
