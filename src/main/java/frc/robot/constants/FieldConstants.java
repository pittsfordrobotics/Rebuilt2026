package frc.robot.constants;

import static edu.wpi.first.apriltag.AprilTagFields.k2026RebuiltWelded;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;


public class FieldConstants {
public static final AprilTagFieldLayout aprilTags;
public static final double fieldLength = 651.22;
public static final double fieldWidth = 317.69;

static {
    try {
        aprilTags = AprilTagFieldLayout.loadFromResource(k2026RebuiltWelded.m_resourceFile);
    } catch(IOException e) {
        throw new RuntimeException(e);
    }
}
}
