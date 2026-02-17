package frc.robot.constants;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.util.AllianceFlipUtil;

public class ClimberConstants {
    public static final int CLIMBER_MOTOR = 41;

    public static final Pose2d BLUE_CLIMB_EXTENDED_POS = new Pose2d(new Translation2d(Units.inchesToMeters(42.075+6), Units.inchesToMeters(147.47)), new Rotation2d(0));
    public static final Pose2d BLUE_CLIMB_UNEXTENDED_POS = new Pose2d(new Translation2d(Units.inchesToMeters(42.075+18), Units.inchesToMeters(147.47)), new Rotation2d(0));

    public static final Supplier<Pose2d> FLIPPED_CLIMB_EXTENDED_POS = () -> AllianceFlipUtil.apply(BLUE_CLIMB_EXTENDED_POS);
    public static final Supplier<Pose2d> FLIPPED_CLIMB_UNEXTENDED_POS = () -> AllianceFlipUtil.apply(BLUE_CLIMB_UNEXTENDED_POS);

}
