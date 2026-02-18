package frc.robot.constants;

import static edu.wpi.first.apriltag.AprilTagFields.k2026RebuiltWelded;

import java.io.IOException;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.util.AllianceFlipUtil;


public class FieldConstants {
    public static final AprilTagFieldLayout aprilTags;
    public static final double fieldLength = Units.inchesToMeters(651.22);
    public static final double fieldWidth = Units.inchesToMeters(317.69);
    public static final Translation2d blueHubPosition = new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
    public static final Supplier<Translation2d> flippedHubPosition = () -> AllianceFlipUtil.apply(blueHubPosition);

    static {
        try {
            aprilTags = AprilTagFieldLayout.loadFromResource(k2026RebuiltWelded.m_resourceFile);
        } catch(IOException e) {
            throw new RuntimeException(e);
        }
    }
}
