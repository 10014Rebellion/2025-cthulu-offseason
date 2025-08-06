package frc.robot.systems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
// import frc.robot.subsystems.drive.DriveConstants;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * RELATIVE TO THE INTAKE. INTAKE/SHOOTING SIDE IS FRONT!
 *
 * <p>Map of camera positions relative to the robot's center.
 *
 * <p>- **Translation3d (X, Y, Z)**: - X: Forward (+) / Backward (-) relative to the center of the
 * bot - Y: Left (+) / Right (-) relative to the center of the bot - Z: Up (+) / Down (-) relative
 * to the ground, most likely wont be inside the ground
 *
 * <p>- **Rotation3d (Roll, Pitch, Yaw)**: - Roll (X-axis rotation): Side tilt (it will prolly be 0
 * unless we do some crazy stuff) - Pitch (Y-axis rotation): Camera looking up/down (Negative = up,
 * positive = down) - Yaw (Z-axis rotation): Camera turning left/right.
 *
 * <p>Imagine a birds eye view of the bot, 0deg is north, 90 is west, -90 is east, and 180 is south
 */
public class VisionConstants {
  // From CAD and decided by you in configuration
  public static final String kLeftCamName = "FrontLeftOV9281";
  public static final Orientation kLeftCamOrientation = Orientation.FRONT;
  public static final Transform3d kLeftCamTransform =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(9.2055), // X: inches forward
              Units.inchesToMeters(13.04025), // Y: inches left
              Units.inchesToMeters(9.02406) // Z: inches above ground
              ),
          new Rotation3d(
              Units.degreesToRadians(0), // Roll: No side tilt
              Units.degreesToRadians(0), // Pitch: No upward tilt
              Units.degreesToRadians(52.5) // Yaw: (angled inward)
              ));

  public static final String kRightCamName = "BackRightOV9281";
  public static final Orientation kRightCamOrientation = Orientation.FRONT;
  public static final Transform3d kRightCamTransform = new Transform3d(
      new Translation3d(
          Units.inchesToMeters(-7.69372), // X: inches forward
          Units.inchesToMeters(-13.0520), // Y: inches left
          Units.inchesToMeters(10.02406) // Z: inches above ground
      ),
      new Rotation3d(
          Units.degreesToRadians(0), // Roll: No side tilt
          Units.degreesToRadians(0), // Pitch: No upward tilt
          Units.degreesToRadians(217.5) // Yaw: (angled inward)
      )
  );

  /* TODO: SET TO FALSE UNLESS YOU ACTUALLY KNOW WHAT THIS DOES
   * This turns on a implementation of single tag vision algorithm that may be more accurate
   * https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/85
   */

  // Tuned by using AdvantageScope data analysis tool(Normal distribution)
  public static final Vector<N3> kSingleStdDevs =
      (RobotBase.isReal())
          ? VecBuilder.fill(0.274375, 0.274375, 5.0)
          : VecBuilder.fill(0.23, 0.23, 5.0);
  public static final Vector<N3> kMultiStdDevs =
      (RobotBase.isReal())
          ? VecBuilder.fill(0.23188, 0.23188, 5.0)
          : VecBuilder.fill(0.23, 0.23, 5.0);

  public static final double kAmbiguityThreshold = (RobotBase.isReal()) ? 0.2 : 1.0;

  public static final Rotation2d kOV2311DiagonalCameraFOV = Rotation2d.fromDegrees(95.0);
  public static final boolean KUseSingleTagTransform = false;

  public static enum Orientation {
    BACK,
    FRONT
  }
}
