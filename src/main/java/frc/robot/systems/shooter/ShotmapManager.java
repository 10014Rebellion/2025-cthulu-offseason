package frc.robot.systems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.systems.arm.Arm;
import frc.robot.systems.drive.Drive;
import frc.robot.utils.ShooterSetpoint;
import frc.robot.utils.ShooterShotmap;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShotmapManager extends SubsystemBase{
    private final double kTEMP_DISTANCE_M = 0;

    private final double[][] shotMapPoints = {
        // Distance, RPM, Angle
        // RPM IS FOR TOP FLYWHEEL.
        // BOTTOM FLYWHEEL HAS A MULTIPLIER
        // NOTE: i was really confused making this it could be swapped tbh
        {4.6, 5800, 12},
        {4.1, 5000, 14},
        {3.6, 4900, 17},
        {3.1, 4600, 18},
        {2.6, 4500, 20},
        {2.1, 4200, 22},
        {1.6, 4200, 30},
        {1.1, 4200, 33} // GOOD FAILSAFE SETPOINT
    };

    private InterpolatingDoubleTreeMap flywheelVelocityMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap shooterAngleMap = new InterpolatingDoubleTreeMap();

    private ShooterSetpoint mCurrentSetpoint = new ShooterSetpoint(0.0, 0.0);

    private final Arm mArm;
    private final Flywheels mFlywheels;
    private final Drive mDrive;
    //private final ShooterShotmap mShotmap;

    public ShotmapManager(Arm pArm, Flywheels pFlywheels, Drive pDrive) {
        this.mArm = pArm;
        this.mFlywheels = pFlywheels;
        this.mDrive = pDrive;
        //this.mShotmap = new ShooterShotmap(shotMapPoints);
        instantiateMaps();
    }

    private void instantiateMaps() {
        for (double[] list : shotMapPoints) {
            flywheelVelocityMap.put(list[0], list[1]);
            shooterAngleMap.put(list[0], list[2]);
        } 
    }

    // private Command shootCommand() {
    //     return
    //         new ParallelCommandGroup(
    //             mArm.setPIDCmd(getShooterSetpoint(kTEMP_DISTANCE_M).mAngle),
    //             mFlywheels.setBothFlywheelsRPM(getShooterSetpoint(kTEMP_DISTANCE_M).mRPM)
    //         );
    // }

    public FunctionalCommand setFlywheelCmd() {
        return new FunctionalCommand(
        () -> {}, 
        () -> {
            mFlywheels.setBothRPM(mCurrentSetpoint.mRPM * FlywheelConstants.topMultiplier, 
            mCurrentSetpoint.mRPM * FlywheelConstants.bottomMultipler);
        }, 
        (interrupted) -> {}, 
        () -> false, this);
    }

    public ShooterSetpoint getShooterSetpoint(){
        return mCurrentSetpoint;
    }

    private void updateShooterSetpoint(double pDistance) {
        this.mCurrentSetpoint = new ShooterSetpoint(flywheelVelocityMap.get(pDistance), shooterAngleMap.get(pDistance));
    }

    public void periodic() {
        updateShooterSetpoint(mDrive.getShotDistance());
        Logger.recordOutput("Shotmap/Arm Setpoint", mCurrentSetpoint.mAngle);
        Logger.recordOutput("Shotmap/Top Setpoint", mCurrentSetpoint.mRPM * FlywheelConstants.topMultiplier);
        Logger.recordOutput("Shotmap/Bottom Setpoint", mCurrentSetpoint.mRPM * FlywheelConstants.bottomMultipler);
        //updateShooterSetpoint();
    }
}