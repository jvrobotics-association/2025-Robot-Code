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
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants.ElevatorHeight;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GamePieceCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AlgaeGrabber;
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

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Elevator elevator;
  private static CoralManipulator coralManipulator;
  private final AlgaeArm algaeManipulator;
  private final AlgaeGrabber algaeGrabber;
  private final Climber climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandJoystick operatorConsole = new CommandJoystick(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private boolean isRelativeDrive = false;

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
    algaeManipulator = new AlgaeArm();
    algaeGrabber = new AlgaeGrabber();
    climber = new Climber();

    // Add named commands for PathPlanner
    NamedCommands.registerCommand(
        "ScoreCoralL2",
        GamePieceCommands.placeCoralCommand(elevator, coralManipulator, ElevatorHeight.L2));
    NamedCommands.registerCommand(
        "ScoreCoralL3",
        GamePieceCommands.placeCoralCommand(elevator, coralManipulator, ElevatorHeight.L3));
    NamedCommands.registerCommand(
        "ScoreCoralL4",
        GamePieceCommands.placeCoralCommand(elevator, coralManipulator, ElevatorHeight.L4));
    NamedCommands.registerCommand(
        "RemoveAlgaeL2",
        GamePieceCommands.collectAlgae(
            drive, elevator, algaeManipulator, algaeGrabber, ElevatorHeight.L2_ALGAE));
    NamedCommands.registerCommand(
        "RemoveAlgaeL3",
        GamePieceCommands.collectAlgae(
            drive, elevator, algaeManipulator, algaeGrabber, ElevatorHeight.L2_ALGAE));
    NamedCommands.registerCommand(
        "ScoreAlgae", GamePieceCommands.scoreAlgae(elevator, algaeManipulator, algaeGrabber));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption("Leave Auto Zone", DriveCommands.leaveAutoZome(drive));

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
            () -> isRelativeDrive,
            () -> elevator.getPosition() >= ElevatorHeight.L2.height ? 4 : 1,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Driver Left Trigger: Align the robot to the nearest left reef pole
    controller
        .leftTrigger(0.25)
        .whileTrue(
            DriveCommands.joystickApproach(
                drive,
                () -> elevator.getPosition() >= ElevatorHeight.L2.height ? 4 : 2.5,
                () -> -controller.getLeftY(),
                () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.LEFT)));

    // Driver Right Trigger: Align the robot to the nearest right reef pole
    controller
        .rightTrigger(0.25)
        .whileTrue(
            DriveCommands.joystickApproach(
                drive,
                () -> elevator.getPosition() >= ElevatorHeight.L2.height ? 4 : 2.5,
                () -> -controller.getLeftY(),
                () -> FieldConstants.getNearestReefBranch(drive.getPose(), ReefSide.RIGHT)));

    // Driver POV Up: Reset robot field orientation to 0º
    controller
        .povUp()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Driver Back Button: Toggle the robot between field relative and robot centric drive modes
    controller.back().toggleOnTrue(new InstantCommand(() -> isRelativeDrive = !isRelativeDrive));

    // Driver Menu Button: Reset Algae Manipulator to the start position and stop the grabber motor
    controller
        .button(8)
        .onTrue(GamePieceCommands.resetAlgaeManipulator(algaeManipulator, algaeGrabber));

    // Driver Y: Align to the processor while held
    controller
        .y()
        .whileTrue(
            DriveCommands.joystickApproach(
                drive,
                () -> elevator.getPosition() >= ElevatorHeight.L2.height ? 4 : 2.5,
                () -> -controller.getLeftY(),
                () -> FieldConstants.getNearestProcessorFace(drive.getPose())));

    // Driver B: Align to the nearest source while held
    controller
        .b()
        .whileTrue(
            DriveCommands.joystickApproach(
                drive,
                () -> elevator.getPosition() >= ElevatorHeight.L2.height ? 4 : 2.5,
                () -> -controller.getLeftY(),
                () -> FieldConstants.getNearestCoralStation(drive.getPose())));

    // Driver A: Align to the nearest reef face in the center for collecting algae
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickApproach(
                drive,
                () -> elevator.getPosition() >= ElevatorHeight.L2.height ? 4 : 2.5,
                () -> -controller.getLeftY(),
                () -> FieldConstants.getNearestReefFace(drive.getPose())));

    // Driver X: Switch to an X pattern to lock the robot in place
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset chute servo to closed
    controller.povRight().onTrue(Commands.run(() -> climber.engageChute(), climber));

    // Lock to 0° when A button is held
    // controller
    //     .b()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // Score algae in the processor
    operatorConsole
        .button(8)
        .onTrue(GamePieceCommands.scoreAlgae(elevator, algaeManipulator, algaeGrabber));

    // Kyra Button :)
    operatorConsole
        .button(4)
        .onTrue(GamePieceCommands.kyra(elevator, algaeManipulator, algaeGrabber));

    // Launch Algae
    operatorConsole
        .button(5)
        .onTrue(GamePieceCommands.launchAlgae(elevator, algaeManipulator, algaeGrabber));

    // Place coral on L2
    operatorConsole
        .button(3)
        .onTrue(GamePieceCommands.placeCoralCommand(elevator, coralManipulator, ElevatorHeight.L2));

    // Collect algae from L2
    operatorConsole
        .button(6)
        .onTrue(
            GamePieceCommands.collectAlgae(
                drive, elevator, algaeManipulator, algaeGrabber, ElevatorHeight.L2_ALGAE));

    // Place coral on L3
    operatorConsole
        .button(1)
        .onTrue(GamePieceCommands.placeCoralCommand(elevator, coralManipulator, ElevatorHeight.L3));

    // Collect algae on L3
    operatorConsole
        .button(7)
        .onTrue(
            GamePieceCommands.collectAlgae(
                drive, elevator, algaeManipulator, algaeGrabber, ElevatorHeight.L3_ALGAE));

    // Place coral on L4
    operatorConsole
        .button(2)
        .onTrue(GamePieceCommands.placeCoralCommand(elevator, coralManipulator, ElevatorHeight.L4));

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
        .button(13)
        .whileTrue(
            Commands.runEnd(
                () -> elevator.manuallyLower(),
                () -> {
                  elevator.holdCurrentPosition();
                },
                elevator));

    // Manually score the coral
    operatorConsole
        .button(10)
        .whileTrue(Commands.run(() -> coralManipulator.setSpeed(0.3), coralManipulator));

    // Extend the climber
    operatorConsole.button(14).onTrue(ClimberCommands.extendClimber(climber));

    // Retract the climber
    operatorConsole.button(15).onTrue(ClimberCommands.retractClimber(climber));

    // DO A FLIP :) WARNING: WILL DO AS MANY BACKFLIPS AS BUTTON PRESSES
    // controller.povDown().onTrue(DriveCommands.backflip());
  }

  public static boolean isCoralDetected() {
    return coralManipulator.getCoralSensorDetected();
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
