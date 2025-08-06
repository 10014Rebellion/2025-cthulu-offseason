package frc.robot.systems.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.systems.arm.Arm;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.shooter.Flywheels;

public class ButtonBindings {
  private final CommandXboxController mDriverController;
  private final CommandXboxController mOperatorController;
  private final TeleopCommands mTeleopCommands;

  private final Drive mDrive;
  private final Arm mArm;
  private final Intake mIntake;
  private final Flywheels mFlywheels;
  // private final LEDSubsystem mLEDs;

  public ButtonBindings(Drive pDrive, Arm pArm, Intake pIntake, Flywheels pFlywheels) {
    this.mDrive = pDrive;
    this.mArm = pArm;
    this.mIntake = pIntake;
    this.mFlywheels = pFlywheels;
    // this.mLEDs = pLEDs;

    this.mDriverController = new CommandXboxController(0);
    this.mOperatorController = new CommandXboxController(1);
    this.mTeleopCommands = new TeleopCommands(pDrive, pArm, pIntake, pFlywheels);
  }

  public void initDriverJoysticks() {
    mDrive.setDefaultCommand(
        DriveCommands.joystickDrive(
            mDrive,
            () -> -mDriverController.getLeftY(),
            () -> -mDriverController.getLeftX(),
            () -> mDriverController.getRightX()));
  }

  public void initTriggers() {}

  // All bindings for the driver controller go here.
  public void initDriverButtons() {
    // Lock to 0° when A button is held
    mDriverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                mDrive,
                () -> -mDriverController.getLeftY(),
                () -> -mDriverController.getLeftX(),
                () -> new Rotation2d()));

    // Reset gyro to 0° when B button is pressed
    mDriverController
        .x()
        .onTrue(
            Commands.runOnce(
                    () ->
                        mDrive.setPose(
                            new Pose2d(mDrive.getPose().getTranslation(), new Rotation2d())),
                    mDrive)
                .ignoringDisable(true));

    mDriverController.rightBumper().whileTrue(mTeleopCommands.getIntakeCoralCmd());

    mDriverController.rightTrigger().whileTrue(mTeleopCommands.getIntakeFloorAlgaeCmd());
  }
  // All bindings for the operator should be here.
  public void initOperatorBindings() {
    mOperatorController.a().whileTrue(mTeleopCommands.scoreProcessorCmd());
  }

  // All bindings for running certain commands that aren't ready for "competition" use yet go here.
  public void initTestBindings() {
    // mDriverController.b().whileTrue(mTeleopCommands.scoreProcessorCmd());

    mDriverController.povUp().whileTrue(mArm.setVoltageCommand(3));

    mDriverController.povDown().whileTrue(mArm.setVoltageCommand(-3));

    mDriverController.b().whileTrue(mIntake.intakeTunablePivotToGoal());

    // mDriverController.y().whileTrue(mArm.setPIDCmd(45));

    // mDriverController.x().whileTrue(mArm.setPIDCmd(0));

    // mDriverController.a().whileTrue(mArm.setPIDCmd(-30));

    mDriverController.rightBumper().whileTrue(mTeleopCommands.getIntakeFloorAlgaeCmd());
    mDriverController.rightTrigger().whileTrue(mFlywheels.setIndexerVoltageCommand(-12));

    mDriverController.leftBumper().whileTrue(mTeleopCommands.getIntakeCoralCmd());

    mDriverController.y().whileTrue(
        mFlywheels.setTuneableRPM());
    mDriverController.a().whileTrue(
      new ParallelCommandGroup(
        mFlywheels.setTopFlywheelRPM(3000),
        mFlywheels.setBottomFlywheelRPM(3000)
    ));
    //mDriverController.b().whileTrue(mFlywheels.setBottomFlywheelRPM(6000));
    mDriverController.x().whileTrue(mArm.setTuneablePIDCmd());
    //mDriverController.a().whileTrue(mFlywheels.setBottomFlywheelRPM(3000));
  }
}
