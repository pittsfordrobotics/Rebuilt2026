package frc.robot.constants;

import frc.robot.subsystems.Vision.VisionIOLimelight;
import frc.robot.subsystems.Vision.VisionIO;

/** Vision CONSTANTS! Do not create limelight or other VisionIO objects here, they need to be assigned in RobotContainer.java */
public final class VisionConstants {

    //Setting up names of limelights
    public final static String LIMELIGHT_LEFT_NAME = "limelight-left";
    public final static String LIMELIGHT_RIGHT_NAME = "limelight-right";
    public final static String LIMELIGHT_FRONT_NAME = "limelight-front";

    public final static VisionIO LIMELIGHT_LEFT = new VisionIOLimelight(LIMELIGHT_LEFT_NAME);
    public final static VisionIO LIMELIGHT_RIGHT = new VisionIOLimelight(LIMELIGHT_RIGHT_NAME);
    public final static VisionIO LIMELIGHT_FRONT = new VisionIOLimelight(LIMELIGHT_FRONT_NAME);
    
    public static final double FIELD_BORDER_MARGIN = 0.5;
    public static final double Z_MARGIN = 0.75;
    public static final double XY_STD_DEV_COEF = 0.2;
    
    //Don't even bother using vision for heading measurement, Pigeon2 is good enough
    public static final double THETA_STD_DEV_COEF = 1;
    public static final double TARGET_LOG_SECONDS = 0.1;
}