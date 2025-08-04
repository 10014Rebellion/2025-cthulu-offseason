package frc.robot.systems.controls;

import frc.robot.systems.drive.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.systems.controls.TeleopCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.systems.arm.Arm;
import frc.robot.systems.arm.ArmConstants;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.IntakeConstants;
import frc.robot.systems.intake.IntakeConstants.Pivot;
import frc.robot.systems.intake.IntakeConstants.Roller;
import frc.robot.systems.shooter.FlywheelConstants;
import frc.robot.systems.shooter.FlywheelConstants.bottomFlywheel;
import frc.robot.systems.shooter.FlywheelConstants.indexer;
import frc.robot.systems.shooter.FlywheelConstants.topFlywheel;
import frc.robot.systems.shooter.Flywheels;

public class TeleopCommands {
    private final Drive mDrive;
    private final Arm mArm;
    private final Intake mIntake;
    private final Flywheels mFlywheels;

    public TeleopCommands(Drive pDrive, Arm pArm, Intake pIntake, Flywheels pFlywheels){
        this.mDrive = pDrive;
        this.mArm = pArm;
        this.mIntake = pIntake;
        this.mFlywheels = pFlywheels;
    }

    public Command getIntakeCoralCmd() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                mIntake.setIntakePivotCmd(Pivot.Setpoints.StowIntake.getPos()),
                mIntake.intakeCoralCmd()),
            mIntake.setIntakePivotCmd(Pivot.Setpoints.StowIntake.getPos())
        );
    }

    public Command getIntakeFloorAlgaeCmd() {
        return new SequentialCommandGroup(
            mIntake.setIntakePivotCmd(Pivot.Setpoints.IntakeAlgae.getPos()),
            new ParallelDeadlineGroup(
                mFlywheels.intakeAlgaeCommand(),
                mArm.setPIDCmd(ArmConstants.Setpoints.Intake.getPos()),
                mIntake.setIntakePivotCmd(Pivot.Setpoints.IntakeAlgae.getPos()),
                mIntake.setRollerVoltageCommand(IntakeConstants.Roller.Voltage.IntakeAlgae.getVoltage())
                ),
            mIntake.setIntakePivotCmd(Pivot.Setpoints.AvoidArm.getPos()),
            mArm.setPIDCmd(ArmConstants.Setpoints.Hold.getPos()),
            mIntake.setIntakePivotCmd(Pivot.Setpoints.StowIntake.getPos())
            );
    }

    // this one is gonna be a doozy to set up. for now i will just put the basic version.
    public Command basicFireAlgaeCmd() {
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

    public Command scoreProcessorCmd() {
        return new SequentialCommandGroup(
            mIntake.setIntakePivotCmd(Pivot.Setpoints.Processor.getPos()),
            mArm.setPIDCmd(ArmConstants.Setpoints.Processor.getPos()),
            new ParallelCommandGroup(
                mFlywheels.setAllVoltageCommand(
                    topFlywheel.Voltage.scoreProcessor.getVoltage(),
                    bottomFlywheel.Voltage.scoreProcessor.getVoltage(),
                    indexer.Voltage.scoreProcessor.getVoltage()
                ),
                mIntake.setRollerVoltageCommand(Roller.Voltage.ScoreProcessor.getVoltage())
            )
        );
    }
}
