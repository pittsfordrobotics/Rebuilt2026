package frc.robot.lib;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record VisionData(Pose2d visionPose, double time, Matrix<N3, N1> visionReliability) {}
