package frc.robot.lib.util;

import java.lang.reflect.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;

public class AllianceFlipUtil {

  /** Flips a translation to the correct side of the field based on the current alliance color. */
  public static Translation2d apply(Translation2d translation) {
    if (shouldFlip()) {
      return new Translation2d(FieldConstants.fieldLength - translation.getX(), FieldConstants.fieldWidth - translation.getY());
    } else {
      return translation;
    }
  }

  /** Flips a rotation based on the current alliance color. */
  public static Rotation2d apply(Rotation2d rotation) {
    if (shouldFlip()) {
      return rotation.plus(Rotation2d.k180deg);
    } else {
      return rotation;
    }
  }

  /** Flips a pose to the correct side of the field based on the current alliance color. */
  public static Pose2d apply(Pose2d pose) {
    if (shouldFlip()) {
      return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
    } else {
      return pose;
    }
  }

  public static Translation3d apply(Translation3d translation3d) {
    if (shouldFlip()) {
      Translation2d translation = apply(translation3d.toTranslation2d());
      return new Translation3d(translation.getX(), translation.getY(), translation3d.getZ());
    } else {
      return translation3d;
    }
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
  }
}
