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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.systems.arm.Arm;
import frc.robot.systems.arm.ArmConstants;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.GyroIO;
import frc.robot.systems.drive.GyroIOPigeon2;
import frc.robot.systems.drive.ModuleIO;
import frc.robot.systems.drive.ModuleIOSim;
import frc.robot.systems.drive.ModuleIOSpark;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.IntakeConstants;
import frc.robot.systems.shooter.Flywheels;
import frc.robot.systems.shooter.ShooterConstants;
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
  private final Arm arm;
  private final Intake intake;
  private final Flywheels flywheels;

  // Controller
  private final CommandXboxController driveController = new CommandXboxController(0);

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
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

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

    arm = new Arm();
    intake = new Intake();
    flywheels = new Flywheels();

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
    configureTestBindings();
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
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> driveController.getRightX()));

    // Lock to 0° when A button is held
    driveController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> new Rotation2d()));

    // Reset gyro to 0° when B button is pressed
    driveController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  private void configureTestBindings() {
    // driveController.rightBumper().whileTrue(arm.setVoltageCommand(6.0));
    // driveController.leftBumper().whileTrue(arm.setVoltageCommand(-6.0));
    driveController
        .rightTrigger()
        .whileTrue(
            new SequentialCommandGroup(
                intake.intakePivotToGoal(IntakeConstants.Pivot.Setpoints.IntakeAlgae.getPos()),
                new ParallelCommandGroup(
                    arm.setPIDCmd(ArmConstants.Setpoints.Intake.getPos().getDegrees()),
                    intake.intakePivotToGoal(IntakeConstants.Pivot.Setpoints.IntakeAlgae.getPos()),
                    intake.setRollerVoltageCommand(
                        IntakeConstants.Roller.Voltage.IntakeAlgae.getVoltage()),
                    flywheels.setTopFlywheelVoltageCommand(
                        ShooterConstants.topFlywheel.Voltage.IntakeAlgae.getVoltage()),
                    flywheels.setBottomFlywheelVoltageCommand(
                        ShooterConstants.bottomFlywheel.Voltage.IntakeAlgae.getVoltage()),
                    flywheels.setIndexerVoltageCommand(
                        ShooterConstants.indexer.Voltage.IndexAlgae.getVoltage()))));
    driveController
        .leftTrigger()
        .whileTrue(
            new ParallelCommandGroup(
                flywheels.setTopFlywheelVoltageCommand(
                    ShooterConstants.topFlywheel.Voltage.BasicShootAlgae.getVoltage()),
                flywheels.setBottomFlywheelVoltageCommand(
                    ShooterConstants.bottomFlywheel.Voltage.BasicShootAlgae.getVoltage()),
                flywheels.setIndexerVoltageCommand(2),
                new SequentialCommandGroup(
                    arm.setPIDCmd(30.0),
                    new WaitCommand(0.5),
                    flywheels.setIndexerVoltageCommand(
                        ShooterConstants.indexer.Voltage.FireAlgae.getVoltage()))));
    driveController
        .rightBumper()
        .whileTrue(
            new ParallelCommandGroup(
                intake.intakePivotToGoal(IntakeConstants.Pivot.Setpoints.IntakeCoral.getPos()),
                intake.setRollerVoltageCommand(
                    IntakeConstants.Roller.Voltage.IntakeCoral.getVoltage())))
        .whileFalse(
            new ParallelCommandGroup(
                intake.intakePivotToGoal(IntakeConstants.Pivot.Setpoints.StowIntake.getPos()),
                intake.setRollerVoltageCommand(
                    IntakeConstants.Roller.Voltage.HoldCoral.getVoltage())));

    driveController
        .leftBumper()
        .whileTrue(
            new ParallelCommandGroup(
                intake.intakePivotToGoal(IntakeConstants.Pivot.Setpoints.ScoreL1.getPos()),
                intake.setRollerVoltageCommand(
                    IntakeConstants.Roller.Voltage.ScoreL1.getVoltage())));
    // driveController.x().whileTrue(arm.setTuneablePIDCmd());
    // driveController.y().whileTrue(arm.enableFFCmd());

    // driveController.povRight().whileTrue(intake.setPivotVoltageCommand(3.0));
    // driveController.povLeft().whileTrue(intake.setPivotVoltageCommand(-3.0));

    driveController.povRight().whileTrue(intake.setRollerVoltageCommand(6.0));
    driveController.povLeft().whileTrue(intake.setRollerVoltageCommand(-6.0));
    driveController.povUp().whileTrue(intake.setPivotVoltageCommand(1.5));
    driveController.povDown().whileTrue(intake.setPivotVoltageCommand(-1.0));

    driveController.x().whileTrue(intake.intakeTunablePivotToGoal());
    driveController.y().whileTrue(intake.intakePivotFF());
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
