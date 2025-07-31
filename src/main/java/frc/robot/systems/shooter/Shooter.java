package frc.robot.systems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ShooterShotmap;

public class Shooter extends SubsystemBase {
  ShooterShotmap mShotmap;

  public Shooter() {
    mShotmap = new ShooterShotmap(ShooterConstants.shooterDatapoints);
  }

  @Override
  public void periodic() {}
}
