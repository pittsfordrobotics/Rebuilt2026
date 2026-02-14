package frc.robot.lib.util;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.AllDeadbands;

public class SwerveHelpers {
    public static Rotation2d getAngleToPoint(Translation2d currentPose, Translation2d targetPose) {
        double desired_heading_rad = Math.atan2(targetPose.getY() - currentPose.getY(),
                targetPose.getX() - currentPose.getX());
        return Rotation2d.fromRadians(desired_heading_rad);
    }

    public static Rotation2d getHeadingFromStick(DoubleSupplier rotationX, DoubleSupplier rotationY) {
        Rotation2d heading = null;
        double[] deadbandRotationInputs = AllDeadbands
                .applyScalingCircularDeadband(new double[] { -rotationX.getAsDouble(), -rotationY.getAsDouble() }, 0.95);
        
        if (deadbandRotationInputs[0] != 0 || deadbandRotationInputs[1] != 0) {
            heading = Rotation2d.fromRadians(Math.atan2(deadbandRotationInputs[1], deadbandRotationInputs[0]));
        }

        return heading;
    }

    public static double[] swerveDeadband(double[] inputs, double deadband) {
        double[] deadbanded = AllDeadbands.applyScalingCircularDeadband(inputs, deadband);
        if(AllianceFlipUtil.isRed()) {
            return deadbanded;
        }

        double[] flipped = new double[deadbanded.length];
        for(int i=0; i<flipped.length; i++) {
            flipped[i] = -deadbanded[i];
        }
        
        return flipped;
    }
}
