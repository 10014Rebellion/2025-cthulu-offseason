package frc.robot.utils;

import edu.wpi.first.math.interpolation.Interpolatable;

public class ShooterSetpoint implements Interpolatable<ShooterSetpoint> {
  public double mRPM;
  public double mAngle;

  public ShooterSetpoint(double pRPM, double pAngle) {
    this.mRPM = pRPM;
    this.mAngle = pAngle;
  }

  @Override
  public ShooterSetpoint interpolate(ShooterSetpoint pSettings, double t) {
    double interpRPM = mRPM + (pSettings.mRPM - mRPM) * t;
    double interpAngle = mAngle + (pSettings.mAngle - mAngle) * t;
    return new ShooterSetpoint(interpRPM, interpAngle);
  }
}
