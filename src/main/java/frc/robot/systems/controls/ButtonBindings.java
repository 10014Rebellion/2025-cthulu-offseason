package frc.robot.systems.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.systems.LEDs.LEDConstants.ledColor;
import frc.robot.systems.LEDs.LEDSubsystem;
import frc.robot.systems.arm.Arm;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.IntakeConstants;
import frc.robot.systems.intake.IntakeConstants.Pivot;
import frc.robot.systems.intake.IntakeConstants.Roller;
import frc.robot.systems.shooter.Flywheels;
import frc.robot.systems.shooter.ShotmapManager;
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
  private final ShotmapManager mShotmapManager;
  private final LEDSubsystem mLEDs;

  public ButtonBindings(Drive pDrive, Arm pArm, Intake pIntake, Flywheels pFlywheels, ShotmapManager pShotmapManager, LEDSubsystem pLEDs) {
    this.mDrive = pDrive;
    this.mArm = pArm;
    this.mIntake = pIntake;
    this.mFlywheels = pFlywheels;
    this.mShotmapManager = pShotmapManager;
    this.mLEDs = pLEDs;

    this.mDriverController = new CommandXboxController(0);
    this.mOperatorController = new CommandXboxController(1);
    this.mTeleopCommands = new TeleopCommands(pDrive, pArm, pIntake, pFlywheels, pShotmapManager);
  }

  public void initDriverJoysticks() {
    mDrive.setDefaultCommand(
        DriveCommands.joystickDrive(
            mDrive,
            () -> -mDriverController.getLeftY(),
            () -> -mDriverController.getLeftX(),
            () -> -mDriverController.getRightX()));
  }

  public void initTriggers() {
    new Trigger(() -> (mDrive.canHitBarge() && mArm.isPIDAtGoal() && mFlywheels.atSetpoint())).debounce(0.1)
      .whileTrue(new InstantCommand(() -> mLEDs.setSolid(ledColor.GREEN)))
      .whileFalse(new InstantCommand(() -> mLEDs.setDefaultColor()));
    
    new Trigger(() -> mFlywheels.getAlgaeDetected())
      .whileTrue(new InstantCommand(() -> mLEDs.setSolid(ledColor.CYAN)))
      .whileFalse(new InstantCommand(() -> mLEDs.setDefaultColor()));
    
      new Trigger(() -> mIntake.hasCoral())
      .whileTrue(new InstantCommand(() -> mLEDs.setSolid(ledColor.YELLOW)))
      .whileFalse(new InstantCommand(() -> mLEDs.setDefaultColor()));

  }

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
            new InstantCommand(() -> mDrive.resetGyro())
            // Commands.runOnce(
            //         () ->
            //             mDrive.setPose(
            //                 new Pose2d(mDrive.getPose().getTranslation(), new Rotation2d())),
            //         mDrive)
                .ignoringDisable(true));

    mDriverController.rightBumper().whileTrue(mTeleopCommands.getIntakeCoralCmd())
    .onFalse(mIntake.setIntakePivotCmd(IntakeConstants.Pivot.Setpoints.StowIntake.getPos()));
    mDriverController.leftBumper().whileTrue(mTeleopCommands.getScoreL1Cmd());

    mDriverController.rightTrigger().whileTrue(mTeleopCommands.getIntakeFloorAlgaeCmd());

    // mDriverController.leftTrigger().whileTrue(mFlywheels.setAllVoltageCommand(
    //   topFlywheel.Voltage.scoreProcessor.getVoltage(),
    //   bottomFlywheel.Voltage.scoreProcessor.getVoltage(),
    //   indexer.Voltage.scoreProcessor.getVoltage()));
  }
  // All bindings for the operator should be here.
  public void initOperatorBindings() {
    mOperatorController.rightBumper().whileTrue(mTeleopCommands.getPrepL1Cmd());

    mOperatorController.leftTrigger().whileTrue(
      mTeleopCommands.getPrepShooterCmd());
    mOperatorController.rightTrigger().onTrue(
      new ParallelDeadlineGroup(new WaitCommand(1.0), 
      mFlywheels.setIndexerVoltageCommand(indexer.Voltage.FireAlgae.getVoltage())));

    mOperatorController.y().whileTrue(mTeleopCommands.getIntakeL2AlgaeCmd());
    mOperatorController.x().whileTrue(mTeleopCommands.getIntakeL3AlgaeCmd());
    mOperatorController.a().whileTrue(mTeleopCommands.getScoreProcessorCmd());
  }

  // All bindings for running certain commands that aren't ready for "competition" use yet go here.
  public void initTestBindings() {
    mDriverController.povUp().whileTrue(
      new InstantCommand(() -> mLEDs.setSolid(ledColor.YELLOW))
    );
    mDriverController.povLeft().whileTrue(
      new InstantCommand(() -> mLEDs.setSolid(ledColor.GREEN))
    );
    mDriverController.povDown().whileTrue(
      new InstantCommand(() -> mLEDs.setSolid(ledColor.CYAN))
    );
    mDriverController.povRight().whileTrue(
      new InstantCommand(() -> mLEDs.setSolid(ledColor.RED))
    );
  }
}
