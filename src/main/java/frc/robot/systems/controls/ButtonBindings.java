package frc.robot.systems.controls;

import frc.robot.systems.drive.Drive;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.systems.arm.Arm;
import frc.robot.systems.intake.Intake;
import frc.robot.systems.shooter.Flywheels;

public class ButtonBindings{
    private final CommandXboxController mDriverController;
    private final CommandGenericHID mOperatorButtonboard;

    private final Drive mDrive;
    private final Arm mArm;
    private final Intake mIntake;
    private final Flywheels mShooter;
    //private final LEDSubsystem mLEDs;

    public ButtonBindings(Drive pDrive, Arm pArm, Intake pIntake, Flywheels pFlywheels){
        this.mDrive = pDrive;
        this.mArm = pArm;
        this.mIntake = pIntake;
        this.mWrist = pWrist;
    //this.mLEDs = pLEDs;
    }
}
