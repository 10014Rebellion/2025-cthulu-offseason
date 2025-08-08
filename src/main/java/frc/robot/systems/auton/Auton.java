package frc.robot.systems.auton;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.systems.LEDs.LEDSubsystem;
import frc.robot.systems.arm.Arm;
import frc.robot.systems.arm.ArmConstants;
import frc.robot.systems.drive.Drive;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.intake.IntakeConstants.Pivot;
import frc.robot.systems.intake.IntakeConstants.Roller;
import frc.robot.systems.shooter.Flywheels;
import frc.robot.systems.shooter.ShotmapManager;
import frc.robot.systems.shooter.FlywheelConstants.indexer;

public class Auton {
    private final Drive mDrive;
    private final Arm mArm;
    private final Intake mIntake;
    private final Flywheels mFlywheels;
    private final ShotmapManager mShotmapManager;
    public Auton(Drive pDrive, Arm pArm, Intake pIntake, Flywheels pFlywheels, ShotmapManager pShotmapManager, LEDSubsystem pLEDs) {
        this.mDrive = pDrive;
        this.mArm = pArm;
        this.mIntake = pIntake;
        this.mFlywheels = pFlywheels;
        this.mShotmapManager = pShotmapManager;
        initNamedCommands();
    }

    public void initNamedCommands() {
        NamedCommands.registerCommand("ReadyAlgaeL3", readyL3AlgaeCmd());
        NamedCommands.registerCommand("ReadyAlgaeL2", readyL2AlgaeCmd());
        NamedCommands.registerCommand("IntakeAlgae", intakeAlgaeCmd());
        NamedCommands.registerCommand("ReadyCoral", readyCoralCmd());
        NamedCommands.registerCommand("ScoreCoral", scoreCoralCmd());
        NamedCommands.registerCommand("ShootAlgae", shootAlgaeCmd());
        NamedCommands.registerCommand("ScoreAlgae", shootAlgaeCmd()); // Just in case we miswrite it.
    }

    public Command readyL3AlgaeCmd(){
        return new ParallelDeadlineGroup(
                mArm.setPIDCmd(ArmConstants.Setpoints.L3Algae.getPos()),
                mIntake.setIntakePivotCmd(Pivot.Setpoints.AvoidArm.getPos())
            );
    }

    public Command readyL2AlgaeCmd(){
        return new ParallelDeadlineGroup(
                mArm.setPIDCmd(ArmConstants.Setpoints.L2Algae.getPos()),
                mIntake.setIntakePivotCmd(Pivot.Setpoints.AvoidArm.getPos())
            );
    }

    public Command intakeAlgaeCmd() {
        return mFlywheels.intakeReefAlgaeCommand();
    }

    public Command readyCoralCmd(){
        return new ParallelDeadlineGroup(
            mIntake.setIntakePivotCmd(Pivot.Setpoints.ScoreL1.getPos()),
            mIntake.setRollerVoltageCommand(Roller.Voltage.HoldCoral.getVoltage())
            );
    }

    public Command scoreCoralCmd() {
        return new ParallelDeadlineGroup(
            mIntake.scoreCoralCmd(),
            mIntake.setIntakePivotCmd(Pivot.Setpoints.ScoreL1.getPos()),
            mIntake.setRollerVoltageCommand(Roller.Voltage.HoldCoral.getVoltage())
            );
    }

    public Command shootAlgaeCmd(){
        return 
        new SequentialCommandGroup(
            new ParallelRaceGroup(
                new WaitCommand(1.0),
                mShotmapManager.alignShooterCommand()
            ),
            new ParallelRaceGroup(
                new WaitCommand(0.5),
                mFlywheels.setIndexerVoltageCommand(indexer.Voltage.FireAlgae.getVoltage())
            )
            
        );
        
    }
}
