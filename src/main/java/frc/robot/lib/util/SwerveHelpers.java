package frc.robot.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveHelpers {
    public static Rotation2d getAngleToPoint(Translation2d currentPose, Translation2d targetPose) {
        double desired_heading_rad = Math.atan2(targetPose.getY() - currentPose.getY(),
                targetPose.getX() - currentPose.getX())+Math.PI;
        return Rotation2d.fromRadians(desired_heading_rad);
    }
}
