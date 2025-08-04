package frc.robot.systems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
// import frc.robot.subsystems.drive.DriveConstants;

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
  public static final String kLeftCamName = "";
  public static final Orientation kLeftCamOrientation = Orientation.BACK;
  public static final Transform3d kLeftCamTransform =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(8.25), // X: inches forward
              Units.inchesToMeters(12.75), // Y: inches left
              Units.inchesToMeters(8.875) // Z: inches above ground
              ),
          new Rotation3d(
              Units.degreesToRadians(0), // Roll: No side tilt
              Units.degreesToRadians(0), // Pitch: No upward tilt
              Units.degreesToRadians(30) // Yaw: (angled inward)
              ));

  // public static final String kRightCamName = "";
  // public static final Orientation kRightCamOrientation = Orientation.BACK;
  // public static final Transform3d kRightCamTransform = new Transform3d(
  //     new Translation3d(
  //         Units.inchesToMeters(), // X: inches forward
  //         Units.inchesToMeters(), // Y: inches right
  //         Units.inchesToMeters() // Z: inches above ground
  //     ),
  //     new Rotation3d(
  //         Units.degreesToRadians(), // Roll: No side tilt
  //         Units.degreesToRadians(), // Pitch: No upward tilt
  //         Units.degreesToRadians() // Yaw: (angled inward)
  //     )
  // );

  /* TODO: SET TO FALSE UNLESS YOU ACTUALLY KNOW WHAT THIS DOES
   * This turns on a implementation of single tag vision algorithm that may be more accurate
   * https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2025-build-thread/477314/85
   */
  public static final boolean KUseSingleTagTransform = false;

  public static enum Orientation {
    BACK,
    FRONT
  }
}
