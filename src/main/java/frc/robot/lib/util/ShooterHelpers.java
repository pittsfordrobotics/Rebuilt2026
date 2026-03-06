package frc.robot.lib.util;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.FieldConstants;

public class ShooterHelpers {
    public static double getHubDistInches(Supplier<Pose2d> currentPose) {
		return Units.metersToInches(currentPose.get().getTranslation().getDistance(FieldConstants.flippedHubPosition.get()));
    }

    public static boolean isPassing(Supplier<Pose2d> currentPose){
      return (AllianceFlipUtil.apply(currentPose.get()).getMeasureX().in(Inches) > FieldConstants.endOfAllianceZoneBump);
    }
}
