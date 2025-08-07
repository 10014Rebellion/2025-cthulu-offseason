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
import frc.robot.systems.intake.IntakeConstants;
import frc.robot.systems.intake.IntakeConstants.Roller;
import frc.robot.systems.shooter.Flywheels;
import frc.robot.systems.shooter.FlywheelConstants.bottomFlywheel;
import frc.robot.systems.shooter.FlywheelConstants.indexer;
import frc.robot.systems.shooter.FlywheelConstants.topFlywheel;

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
    mOperatorController.rightBumper().whileTrue(mTeleopCommands.getPrepL1Cmd());

    mOperatorController.y().whileTrue(mTeleopCommands.getIntakeL2AlgaeCmd());
    mOperatorController.x().whileTrue(mTeleopCommands.getIntakeL3AlgaeCmd());
    mOperatorController.a().whileTrue(mTeleopCommands.getScoreProcessorCmd());
  }

  // All bindings for running certain commands that aren't ready for "competition" use yet go here.
  public void initTestBindings() {
    // mDriverController.b().whileTrue(mTeleopCommands.scoreProcessorCmd());

    // mDriverController.povUp().whileTrue(mArm.setVoltageCommand(3));

    // mDriverController.povDown().whileTrue(mArm.setVoltageCommand(-3));

    mDriverController.b().whileTrue(mIntake.intakeTunablePivotToGoal());

    // mDriverController.y().whileTrue(mArm.setPIDCmd(45));

    // mDriverController.x().whileTrue(mArm.setPIDCmd(0));

    // mDriverController.a().whileTrue(mArm.setPIDCmd(-30));

    // mDriverController.leftTrigger().whileTrue(mTeleopCommands.getIntakeFloorAlgaeCmd());
    // mDriverController.leftTrigger().whileTrue(mTeleopCommands.get;

    mDriverController.rightBumper().whileTrue(mTeleopCommands.getIntakeCoralCmd())
    .onFalse(mIntake.setIntakePivotCmd(IntakeConstants.Pivot.Setpoints.StowIntake.getPos()));
    mDriverController.leftBumper().whileTrue(mTeleopCommands.getScoreL1Cmd());

    mDriverController.y().whileTrue(mArm.setTuneablePIDCmd());
    mDriverController.rightTrigger().whileTrue(mTeleopCommands.getIntakeFloorAlgaeCmd());
    mDriverController.leftTrigger().whileTrue(mTeleopCommands.getScoreProcessorCmd());
    mDriverController.povUp().whileTrue(mFlywheels.setAllVoltageCommand(
      topFlywheel.Voltage.IntakeAlgae.getVoltage(),
      bottomFlywheel.Voltage.IntakeAlgae.getVoltage(),
      indexer.Voltage.IndexAlgae.getVoltage()));
    mDriverController.povDown().whileTrue(mFlywheels.setAllVoltageCommand(
      topFlywheel.Voltage.scoreProcessor.getVoltage(),
      bottomFlywheel.Voltage.scoreProcessor.getVoltage(),
      indexer.Voltage.scoreProcessor.getVoltage()));
    // mDriverController.y().whileTrue(
    //     mFlywheels.setTuneableRPM());
    // mDriverController.a().whileTrue(
    //   new ParallelCommandGroup(
    //     mFlywheels.setTopFlywheelRPM(3000),
    //     mFlywheels.setBottomFlywheelRPM(3000)
    // ));
    //mDriverController.b().whileTrue(mFlywheels.setBottomFlywheelRPM(6000));
    // mDriverController.x().whileTrue(mIntake.intakePivotFF());
    // mDriverController.y().whileTrue(mIntake.intakeTunablePivotToGoal());
    // mDriverController.rightBumper().whileTrue(mIntake.intakeCoralCmd());
    // mDriverController.leftBumper().whileTrue(mIntake.setRollerVoltageCommand(Roller.Voltage.ScoreL1.getVoltage()));
    //mDriverController.a().whileTrue(mFlywheels.setBottomFlywheelRPM(3000));
  }
}
