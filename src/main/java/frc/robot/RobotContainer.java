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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.systems.LEDs.LEDSubsystem;
import frc.robot.systems.arm.Arm;
import frc.robot.systems.controls.ButtonBindings;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.drive.GyroIO;
import frc.robot.systems.drive.GyroIOPigeon2;
import frc.robot.systems.drive.ModuleIO;
import frc.robot.systems.drive.ModuleIOSim;
import frc.robot.systems.drive.ModuleIOSpark;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.shooter.Flywheels;
import frc.robot.systems.shooter.ShotmapManager;
import frc.robot.systems.vision.CameraIO;
import frc.robot.systems.vision.CameraIOPV;
import frc.robot.systems.vision.Vision;
import frc.robot.systems.vision.VisionConstants;
import frc.robot.systems.vision.VisionConstants.Orientation;

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
  private final ShotmapManager shotmapManager;
  private final LEDSubsystem LEDs;

  // Controller
  private final ButtonBindings mButtonBindings;

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
                new ModuleIOSpark(3),
                new Vision(new CameraIO[] {
                    new CameraIOPV(VisionConstants.kLeftCamName, VisionConstants.kLeftCamTransform,  VisionConstants.kLeftCamOrientation),
                    new CameraIOPV(VisionConstants.kRightCamName, VisionConstants.kRightCamTransform, VisionConstants.kRightCamOrientation)
                }));

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new Vision(new CameraIO[] {
                  new CameraIOPV(VisionConstants.kLeftCamName, VisionConstants.kLeftCamTransform,  VisionConstants.kLeftCamOrientation),
                  new CameraIOPV(VisionConstants.kRightCamName, VisionConstants.kRightCamTransform, VisionConstants.kRightCamOrientation)
                }));

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new Vision(new CameraIO[] {
                  new CameraIOPV(VisionConstants.kLeftCamName, VisionConstants.kLeftCamTransform,  VisionConstants.kLeftCamOrientation),
                  new CameraIOPV(VisionConstants.kRightCamName, VisionConstants.kRightCamTransform, VisionConstants.kRightCamOrientation)
                }));

        break;
    }

    arm = new Arm();
    arm.setDefaultCommand(arm.enableFFCmd());
    intake = new Intake();
    intake.setDefaultCommand(intake.intakePivotFF());
    flywheels = new Flywheels();
    shotmapManager = new ShotmapManager(arm, flywheels, drive);
    LEDs = new LEDSubsystem();

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

    mButtonBindings = new ButtonBindings(drive, arm, intake, flywheels, shotmapManager, LEDs);
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
    mButtonBindings.initDriverJoysticks();
    mButtonBindings.initDriverButtons();

    // mButtonBindings.initOperatorBindings();

    mButtonBindings.initTestBindings();
  }

  // private void configureTestBindings() {

  //   driveController.povRight().whileTrue(intake.setRollerVoltageCommand(6.0));
  //   driveController.povLeft().whileTrue(intake.setRollerVoltageCommand(-6.0));
  //   driveController.povUp().whileTrue(intake.setPivotVoltageCommand(1.5));
  //   driveController.povDown().whileTrue(intake.setPivotVoltageCommand(-1.0));

  //   driveController.x().whileTrue(intake.intakeTunablePivotToGoal());
  //   driveController.y().whileTrue(intake.intakePivotFF());
  // }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
