package frc.robot.systems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.systems.arm.Arm;
import frc.robot.utils.ShooterSetpoint;
import frc.robot.utils.ShooterShotmap;

public class ShotmapManager {
    private final double kTEMP_DISTANCE_M = 0;

    private final double[][] shotMapPoints = {
        // Distance, RPM, Angle
        {4, 5600, 10},
        {3, 5600, 20}
    };

    private ShooterSetpoint mCurrentSetpoint;

    private final Arm mArm;
    private final Flywheels mFlywheels;
    private final ShooterShotmap mShotmap;

    public ShotmapManager(Arm pArm, Flywheels pFlywheels) {
        this.mArm = pArm;
        this.mFlywheels = pFlywheels;
        this.mShotmap = new ShooterShotmap(shotMapPoints);
    }

    public Command shootCommand() {
        return
            new ParallelCommandGroup(
                mArm.setPIDCmd(getShooterSetpoint(kTEMP_DISTANCE_M).mAngle),
                mFlywheels.setBothFlywheelsRPM(getShooterSetpoint(kTEMP_DISTANCE_M).mRPM)
            );
    }

    public ShooterSetpoint getShooterSetpoint(double pDistance){
        return mShotmap.getSetpoints(pDistance);
    }
}