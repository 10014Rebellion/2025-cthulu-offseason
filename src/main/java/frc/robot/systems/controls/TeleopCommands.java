package frc.robot.systems.controls;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.systems.arm.Arm;
import frc.robot.systems.arm.ArmConstants;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.IntakeConstants;
import frc.robot.systems.intake.IntakeConstants.Pivot;
import frc.robot.systems.intake.IntakeConstants.Roller;
import frc.robot.systems.shooter.FlywheelConstants;
import frc.robot.systems.shooter.FlywheelConstants.bottomFlywheel;
import frc.robot.systems.shooter.FlywheelConstants.indexer;
import frc.robot.systems.shooter.FlywheelConstants.topFlywheel;
import frc.robot.systems.shooter.Flywheels;
import frc.robot.systems.shooter.ShotmapManager;

public class TeleopCommands {
  private final Drive mDrive;
  private final Arm mArm;
  private final Intake mIntake;
  private final Flywheels mFlywheels;
  private final ShotmapManager mShotmapManager;

  public TeleopCommands(Drive pDrive, Arm pArm, Intake pIntake, Flywheels pFlywheels, ShotmapManager pShotmapManager) {
    this.mDrive = pDrive;
    this.mArm = pArm;
    this.mIntake = pIntake;
    this.mFlywheels = pFlywheels;
    this.mShotmapManager = pShotmapManager;
  }

  public Command getIntakeCoralCmd() {
    return new SequentialCommandGroup(
        new ParallelCommandGroup(
            mIntake.setIntakePivotCmd(Pivot.Setpoints.IntakeCoral.getPos()),
            mIntake.intakeCoralCmd()),
        mIntake.setIntakePivotCmd(Pivot.Setpoints.StowIntake.getPos()));
  }

  public Command getScoreL1Cmd() {
    return
        new ParallelCommandGroup(
            mIntake.setIntakePivotCmd(Pivot.Setpoints.ScoreL1.getPos()),
            mIntake.setRollerVoltageCommand(Roller.Voltage.ScoreL1.getVoltage()));
  }

  public Command getPrepL1Cmd() {
    return mIntake.setIntakePivotCmd(Pivot.Setpoints.ScoreL1.getPos());
  }

  public Command getIntakeFloorAlgaeCmd() {
    return new SequentialCommandGroup(
        mIntake.setIntakePivotCmd(Pivot.Setpoints.IntakeAlgae.getPos()),
        new ParallelDeadlineGroup(
            mFlywheels.intakeAlgaeCommand(),
            mArm.setPIDCmd(ArmConstants.Setpoints.Intake.getPos()),
            mIntake.setIntakePivotCmd(Pivot.Setpoints.IntakeAlgae.getPos()),
            mIntake.setRollerVoltageCommand(
                IntakeConstants.Roller.Voltage.IntakeAlgae.getVoltage())),
        mIntake.setIntakePivotCmd(Pivot.Setpoints.AvoidArm.getPos()),
        mArm.setPIDCmd(ArmConstants.Setpoints.Hold.getPos()),
        mIntake.setIntakePivotCmd(Pivot.Setpoints.StowIntake.getPos()));
  }

  // this one is gonna be a doozy to set up. for now i will just put the basic version.
  public Command getBasicFireAlgaeCmd() {
    return new ParallelCommandGroup(
        mFlywheels.setTopFlywheelVoltageCommand(
            FlywheelConstants.topFlywheel.Voltage.BasicShootAlgae.getVoltage()),
        mFlywheels.setBottomFlywheelVoltageCommand(
            FlywheelConstants.bottomFlywheel.Voltage.BasicShootAlgae.getVoltage()),
        mFlywheels.setIndexerVoltageCommand(2),
        new SequentialCommandGroup(
            mArm.setPIDCmd(30.0),
            new WaitCommand(0.5),
            mFlywheels.setIndexerVoltageCommand(
                FlywheelConstants.indexer.Voltage.FireAlgae.getVoltage())));
  }

  public Command getScoreProcessorCmd() {
    return new SequentialCommandGroup(
        mIntake.setIntakePivotCmd(Pivot.Setpoints.Processor.getPos()),
        mArm.setPIDCmd(ArmConstants.Setpoints.Processor.getPos()),
        new ParallelCommandGroup(
            mFlywheels.setAllVoltageCommand(
                topFlywheel.Voltage.scoreProcessor.getVoltage(),
                bottomFlywheel.Voltage.scoreProcessor.getVoltage(),
                indexer.Voltage.scoreProcessor.getVoltage()),
            mIntake.setRollerVoltageCommand(Roller.Voltage.ScoreProcessor.getVoltage())));
  }

  public Command getIntakeL2AlgaeCmd() {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            mFlywheels.intakeReefAlgaeCommand(),
            mArm.setPIDCmd(ArmConstants.Setpoints.L2Algae.getPos()),
            mIntake.setIntakePivotCmd(Pivot.Setpoints.AvoidArm.getPos())
        ),
        mArm.setPIDCmd(ArmConstants.Setpoints.Hold.getPos())
    );
  }

  public Command getIntakeL3AlgaeCmd() {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            mFlywheels.intakeReefAlgaeCommand(),
            mArm.setPIDCmd(ArmConstants.Setpoints.L3Algae.getPos()),
            mIntake.setIntakePivotCmd(Pivot.Setpoints.AvoidArm.getPos())
        ),
        mArm.setPIDCmd(ArmConstants.Setpoints.Hold.getPos())
    );
  }

  public Command getPrepShooterCmd() {
    return new ParallelCommandGroup(
        mIntake.setIntakePivotCmd(Pivot.Setpoints.AvoidArm.getPos()),
        mShotmapManager.alignShooterCommand()
    );
  }
}
