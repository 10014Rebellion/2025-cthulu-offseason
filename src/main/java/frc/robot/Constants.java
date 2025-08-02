package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final boolean kTuningMode = true;
  public static final String kCTRECanBusName = "rio";
  public static final double kRobotVoltage = 12.0;

  public static enum Mode {
    REAL,

    SIM,

    REPLAY
  }
}
