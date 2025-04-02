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
import frc.robot.FieldConstants.ReefAlignLocation;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GamePieceCommands;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.autoAlignCommands.AlignCenterReefCommand;
import frc.robot.commands.autoAlignCommands.AlignLeftBranchCommand;
import frc.robot.commands.autoAlignCommands.AlignProcessorCommand;
import frc.robot.commands.autoAlignCommands.AlignRightBranchCommand;
import frc.robot.commands.autoAlignCommands.AlignSourceCommand;
import frc.robot.commands.autos.OneCoralAuto;
import frc.robot.commands.autos.TestPathAuto;
import frc.robot.commands.autos.ThreeCoralAuto;
import frc.robot.commands.autos.TwoCoralAuto;
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
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;
import java.util.stream.Collectors;
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
  private final LoggedDashboardChooser<String> autoTypeSelector =
      new LoggedDashboardChooser<>("Auto Type");
  private final LoggedDashboardChooser<String> startPositionSelector =
      new LoggedDashboardChooser<>("Starting Position");
  private final LoggedDashboardChooser<String> firstReefFaceSelector =
      new LoggedDashboardChooser<>("First Reef Face");
  private final LoggedDashboardChooser<String> leftRightFirstFaceSelector =
      new LoggedDashboardChooser<>("Left-Right to First Face");
  private final LoggedDashboardChooser<String> firstScoringPositionSelector =
      new LoggedDashboardChooser<>("First Scoring Position");
  private final LoggedDashboardChooser<String> firstScoringHeightSelector =
      new LoggedDashboardChooser<>("First Scoring Height");
  private final LoggedDashboardChooser<String> firstSourceSelector =
      new LoggedDashboardChooser<>("First Source");
  private final LoggedDashboardChooser<String> leftRightFirstSourceSelector =
      new LoggedDashboardChooser<>("Left-Right to First Source");
  private final LoggedDashboardChooser<String> secondReefFaceSelector =
      new LoggedDashboardChooser<>("Second Reef Face");
  private final LoggedDashboardChooser<String> leftRightSecondFaceSelector =
      new LoggedDashboardChooser<>("Left-Right to Second Face");
  private final LoggedDashboardChooser<String> secondScoringPositionSelector =
      new LoggedDashboardChooser<>("Second Scoring Position");
  private final LoggedDashboardChooser<String> secondScoringHeightSelector =
      new LoggedDashboardChooser<>("Second Scoring Height");
  private final LoggedDashboardChooser<String> secondSourceSelector =
      new LoggedDashboardChooser<>("Second Source");
  private final LoggedDashboardChooser<String> leftRightSecondSourceSelector =
      new LoggedDashboardChooser<>("Left-Right to Second Source");
  private final LoggedDashboardChooser<String> thirdReefFaceSelector =
      new LoggedDashboardChooser<>("Third Reef Face");
  private final LoggedDashboardChooser<String> leftRightThirdFaceSelector =
      new LoggedDashboardChooser<>("Left-Right to Third Face");
  private final LoggedDashboardChooser<String> thirdScoringPositionSelector =
      new LoggedDashboardChooser<>("Third Scoring Position");
  private final LoggedDashboardChooser<String> thirdScoringHeightSelector =
      new LoggedDashboardChooser<>("Third Scoring Height");

  private final LoggedDashboardChooser<String> testPathSelector =
      new LoggedDashboardChooser<>("Path to Test");

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

        // Set simulator position to blue left side start
        drive.setPose(new Pose2d(7.5, 7.0, Rotation2d.fromDegrees(180)));
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

    startPositionSelector.addDefaultOption("None", "");
    startPositionSelector.addOption("Left", "LS");
    startPositionSelector.addOption("Middle", "MS");
    startPositionSelector.addOption("Right", "RS");

    autoTypeSelector.addDefaultOption("None", "");
    autoTypeSelector.addOption("One Coral", "1C");
    autoTypeSelector.addOption("Two Coral", "2C");
    autoTypeSelector.addOption("Three Coral", "3C");
    autoTypeSelector.addOption("Drive Forwards Only", "DRIVE");
    autoTypeSelector.addOption("Bump", "BUMP");
    autoTypeSelector.addOption("Test Path", "TESTPATH");

    firstReefFaceSelector.addDefaultOption("None", "");
    firstReefFaceSelector.addOption("One", "F1");
    firstReefFaceSelector.addOption("Two", "F2");
    firstReefFaceSelector.addOption("Three", "F3");
    firstReefFaceSelector.addOption("Four", "F4");
    firstReefFaceSelector.addOption("Five", "F5");
    firstReefFaceSelector.addOption("Six", "F6");

    leftRightFirstFaceSelector.addDefaultOption("None", "");
    leftRightFirstFaceSelector.addOption("Left", "LEFT");
    leftRightFirstFaceSelector.addOption("Right", "RIGHT");

    firstScoringPositionSelector.addDefaultOption("None", "");
    firstScoringPositionSelector.addOption("Left", "LEFT");
    firstScoringPositionSelector.addOption("Center", "CENTER");
    firstScoringPositionSelector.addOption("Right", "RIGHT");

    firstScoringHeightSelector.addDefaultOption("None", "");
    firstScoringHeightSelector.addOption("L1", "L1");
    firstScoringHeightSelector.addOption("L2", "L2");
    firstScoringHeightSelector.addOption("L3", "L3");
    firstScoringHeightSelector.addOption("L4", "L4");

    firstSourceSelector.addDefaultOption("None", "");
    firstSourceSelector.addOption("Left Source", "LSOURCE");
    firstSourceSelector.addOption("Right Source", "RSOURCE");

    leftRightFirstSourceSelector.addDefaultOption("None", "");
    leftRightFirstSourceSelector.addOption("Left", "LEFT");
    leftRightFirstSourceSelector.addOption("Right", "RIGHT");

    secondReefFaceSelector.addDefaultOption("None", "");
    secondReefFaceSelector.addOption("One", "F1");
    secondReefFaceSelector.addOption("Two", "F2");
    secondReefFaceSelector.addOption("Three", "F3");
    secondReefFaceSelector.addOption("Four", "F4");
    secondReefFaceSelector.addOption("Five", "F5");
    secondReefFaceSelector.addOption("Six", "F6");

    leftRightSecondFaceSelector.addDefaultOption("None", "");
    leftRightSecondFaceSelector.addOption("Left", "LEFT");
    leftRightSecondFaceSelector.addOption("Right", "RIGHT");

    secondScoringPositionSelector.addDefaultOption("None", "");
    secondScoringPositionSelector.addOption("Left", "LEFT");
    secondScoringPositionSelector.addOption("Center", "CENTER");
    secondScoringPositionSelector.addOption("Right", "RIGHT");

    secondScoringHeightSelector.addDefaultOption("None", "");
    secondScoringHeightSelector.addOption("L1", "L1");
    secondScoringHeightSelector.addOption("L2", "L2");
    secondScoringHeightSelector.addOption("L3", "L3");
    secondScoringHeightSelector.addOption("L4", "L4");

    secondSourceSelector.addDefaultOption("None", "");
    secondSourceSelector.addOption("Left Source", "LSOURCE");
    secondSourceSelector.addOption("Right Source", "RSOURCE");

    leftRightSecondSourceSelector.addDefaultOption("None", "");
    leftRightSecondSourceSelector.addOption("Left", "LEFT");
    leftRightSecondSourceSelector.addOption("Right", "RIGHT");

    thirdReefFaceSelector.addDefaultOption("None", "");
    thirdReefFaceSelector.addOption("One", "F1");
    thirdReefFaceSelector.addOption("Two", "F2");
    thirdReefFaceSelector.addOption("Three", "F3");
    thirdReefFaceSelector.addOption("Four", "F4");
    thirdReefFaceSelector.addOption("Five", "F5");
    thirdReefFaceSelector.addOption("Six", "F6");

    leftRightThirdFaceSelector.addDefaultOption("None", "");
    leftRightThirdFaceSelector.addOption("Left", "LEFT");
    leftRightThirdFaceSelector.addOption("Right", "RIGHT");

    thirdScoringPositionSelector.addDefaultOption("None", "");
    thirdScoringPositionSelector.addOption("Left", "LEFT");
    thirdScoringPositionSelector.addOption("Center", "CENTER");
    thirdScoringPositionSelector.addOption("Right", "RIGHT");

    thirdScoringHeightSelector.addDefaultOption("None", "");
    thirdScoringHeightSelector.addOption("L1", "L1");
    thirdScoringHeightSelector.addOption("L2", "L2");
    thirdScoringHeightSelector.addOption("L3", "L3");
    thirdScoringHeightSelector.addOption("L4", "L4");

    testPathSelector.addDefaultOption("None", "");
    try {
      List<String> pathFiles =
          Files.walk(Paths.get("/home/lvuser/deploy"))
              .filter(p -> p.toString().endsWith(".path"))
              .map(p -> p.getFileName().toString().replace(".path", ""))
              .collect(Collectors.toList());

      pathFiles.forEach(path -> testPathSelector.addOption(path, path));
    } catch (IOException e) {
      e.printStackTrace();
    }

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
    controller.leftTrigger(0.25).whileTrue(new AlignLeftBranchCommand(drive));

    // Driver Right Trigger: Align the robot to the nearest right reef pole
    controller.rightTrigger(0.25).whileTrue(new AlignRightBranchCommand(drive));

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

    // Driver Y: Align to the nearest processor
    controller.y().whileTrue(new AlignProcessorCommand(drive));

    // Driver B: Align to the nearest source while held
    controller.b().whileTrue(new AlignSourceCommand(drive));

    // Driver A: Align to the nearest reef face in the center for collecting algae
    controller.a().whileTrue(new AlignCenterReefCommand(drive));

    // Driver X: Switch to an X pattern to lock the robot in place
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset chute servo to closed
    controller.povDown().onTrue(Commands.run(() -> climber.engageChute(), climber));

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
        .onTrue(
            Commands.sequence(
                new MoveElevator(elevator, ElevatorHeight.L1),
                Commands.runOnce(() -> elevator.stopElevator(), elevator)));

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
    Command autoCommand = Commands.none();

    if (autoTypeSelector.get().equals("1C")) {
      autoCommand =
          new OneCoralAuto(
              elevator,
              coralManipulator,
              startPositionSelector.get()
                  + "-"
                  + firstReefFaceSelector.get()
                  + "-"
                  + leftRightFirstFaceSelector.get(),
              ReefAlignLocation.valueOf(firstScoringPositionSelector.get()),
              ElevatorHeight.valueOf(firstScoringHeightSelector.get()));
    } else if (autoTypeSelector.get().equals("2C")) {
      autoCommand =
          new TwoCoralAuto(
              elevator,
              coralManipulator,
              startPositionSelector.get()
                  + "-"
                  + firstReefFaceSelector.get()
                  + "-"
                  + leftRightFirstFaceSelector.get(),
              ReefAlignLocation.valueOf(firstScoringPositionSelector.get()),
              ElevatorHeight.valueOf(firstScoringHeightSelector.get()),
              firstReefFaceSelector.get()
                  + "-"
                  + firstSourceSelector.get()
                  + "-"
                  + leftRightFirstSourceSelector.get(),
              firstSourceSelector.get()
                  + "-"
                  + secondReefFaceSelector.get()
                  + "-"
                  + leftRightSecondFaceSelector.get(),
              ReefAlignLocation.valueOf(secondScoringPositionSelector.get()),
              ElevatorHeight.valueOf(secondScoringHeightSelector.get()));
    } else if (autoTypeSelector.get().equals("3C")) {
      autoCommand =
          new ThreeCoralAuto(
              elevator,
              coralManipulator,
              startPositionSelector.get()
                  + "-"
                  + firstReefFaceSelector.get()
                  + "-"
                  + leftRightFirstFaceSelector.get(),
              ReefAlignLocation.valueOf(firstScoringPositionSelector.get()),
              ElevatorHeight.valueOf(firstScoringHeightSelector.get()),
              firstReefFaceSelector.get()
                  + "-"
                  + firstSourceSelector.get()
                  + "-"
                  + leftRightFirstSourceSelector.get(),
              firstSourceSelector.get()
                  + "-"
                  + secondReefFaceSelector.get()
                  + "-"
                  + leftRightSecondFaceSelector.get(),
              ReefAlignLocation.valueOf(secondScoringPositionSelector.get()),
              ElevatorHeight.valueOf(secondScoringHeightSelector.get()),
              secondReefFaceSelector.get()
                  + "-"
                  + secondSourceSelector.get()
                  + "-"
                  + leftRightSecondSourceSelector.get(),
              secondSourceSelector.get()
                  + "-"
                  + thirdReefFaceSelector.get()
                  + "-"
                  + leftRightThirdFaceSelector.get(),
              ReefAlignLocation.valueOf(thirdScoringPositionSelector.get()),
              ElevatorHeight.valueOf(thirdScoringHeightSelector.get()));
    } else if (autoTypeSelector.get().equals("DRIVE")) {
      autoCommand = DriveCommands.leaveAutoZome(drive);
    } else if (autoTypeSelector.get().equals("TESTPATH")) {
      autoCommand = new TestPathAuto(testPathSelector.get());
    } else if (autoTypeSelector.get().equals("BUMP")) {
      autoCommand =
          Commands.sequence(
              DriveCommands.bump(drive),
              new AlignRightBranchCommand(drive),
              GamePieceCommands.placeCoralCommand(elevator, coralManipulator, ElevatorHeight.L4));
    }

    return autoCommand;
  }
}
