package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.lib.VisionData;
import frc.robot.subsystems.Vision.VisionIO.Pipelines;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Timer;

public class Vision extends SubsystemBase {

    // Initialization
    private boolean useVision = true;
    private Consumer<VisionData> visionDataConsumer;
    private Supplier<Rotation2d> gyroangle;
    private Supplier<Double> robotRotationalVelocity;
    private double xyStdDev = 200;

    private final VisionIO[] io;
    private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();
    private Pipelines pipeline = Pipelines.Test;

    StructArrayPublisher<Pose2d> visionPoseArrayPublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("Vision Poses", Pose2d.struct).publish();

    public Vision(Supplier<Rotation2d> gyroangle, Supplier<Double> robotRotationalVelocity, 
    Consumer<VisionData> visionDataConsumer, VisionIO... limelights) {
        this.visionDataConsumer = visionDataConsumer;
        this.gyroangle = gyroangle;
        this.robotRotationalVelocity = robotRotationalVelocity;
        io = limelights;
        FieldConstants.aprilTags.getTags().forEach((AprilTag tag) -> lastTagDetectionTimes.put(tag.ID, 0.0));

        Shuffleboard.getTab("Vision").addBoolean("Is Vison Being Used?", this::usingVision);
        Shuffleboard.getTab("Vision").add("UseVisionToggle", Commands.runOnce(this::useVisionToggle).ignoringDisable(true));

        for (int i = 0; i < io.length; i++) {
            int number = i; // capture a copy of the int to use in the lambda
            Shuffleboard.getTab("Vision").addDouble(i + "/AvgTagDist", () -> this.inputs[number].avgTagDist);
            Shuffleboard.getTab("Vision").addInteger(i + "/NumTags", () -> this.inputs[number].tagCount);
            Shuffleboard.getTab("Vision").addDoubleArray(i + "/TagDistances", () -> this.inputs[number].tagDistances);
            Shuffleboard.getTab("Vision").addString(i + "/TagIDs", () -> Arrays.toString(this.inputs[number].tagIDs)); 
            Shuffleboard.getTab("Vision").addDouble(i + "/Pose2d_X", () -> this.inputs[number].pose.getX());
            Shuffleboard.getTab("Vision").addDouble(i + "/Pose2d_Y", () -> this.inputs[number].pose.getY());
            Shuffleboard.getTab("Vision").addDouble(i + "/Pose2d_Theta",
                    () -> this.inputs[number].pose.getRotation().getDegrees());
        }
        
        Shuffleboard.getTab("Vision").addDouble("XY_std", this::getXYstdDev);
    }

    private final VisionIO.VisionIOInputs[] inputs = new VisionIO.VisionIOInputs[] { 
        new VisionIO.VisionIOInputs(),
        new VisionIO.VisionIOInputs(),
        new VisionIO.VisionIOInputs() };

    public void setUseVision(boolean usevision) {
        this.useVision = usevision;
    }

    public void useVisionToggle() {
        this.useVision = !this.useVision;
    }

    public boolean usingVision() {
        return useVision;
    }

    public void setPipeline(Pipelines pipeline) {
        this.pipeline = pipeline;
    }

    public double getXYstdDev() {
        return xyStdDev;
    }

    @Override
    public void periodic() {
        if (!useVision) {
            return;
        }

        for (int i = 0; i < io.length; i++) {
            // update the inputs from the netwrork tables named camNames[i]
            io[i].updateInputs(inputs[i], gyroangle.get().getDegrees());
            // keeps the pipeline always the same
            // ** No idea why this needs to be set on every periodic.
            //    The pipeline was already set in the VisionIOLimelight constructor.
            io[i].setPipeline(pipeline);
        }

      
        List<Pose2d> allRobotPoses = new ArrayList<>();

        // Pose estimation
        for (int i = 0; i < io.length; i++) {

            // gets the pose
            Pose2d visionCalcPose = inputs[i].pose;

            // if the bot is not connected, or the bot is at the origin, skip
            if (visionCalcPose.equals(new Pose2d()) || !inputs[i].connected) {
                continue;
            }

            // exit if off the field (might be bad)
            if (visionCalcPose.getX() < -VisionConstants.FIELD_BORDER_MARGIN
                    || visionCalcPose.getX() > FieldConstants.fieldLength + VisionConstants.FIELD_BORDER_MARGIN
                    || visionCalcPose.getY() < -VisionConstants.FIELD_BORDER_MARGIN
                    || visionCalcPose.getY() > FieldConstants.fieldWidth + VisionConstants.FIELD_BORDER_MARGIN) {
                continue;
            }

            // exit if the gyro does not match the vision
            double gyroAngle = gyroangle.get().getDegrees();
            if (Math.abs(gyroAngle - visionCalcPose.getRotation().getDegrees()) > 5) {
                System.out.println("Gyro and Vision do not match");
                // continue;
            }

            // exit if the robot is rotating too fast
            if (Math.abs(robotRotationalVelocity.get()) > 6.28) {
                continue;
            }

            if (inputs[i].tagCount == 0) {
                continue;
            }

            // Get tag poses and update last detection times
            List<Pose3d> tagPoses = new ArrayList<>();
            for (int z = 0; z < inputs[i].tagIDs.length; z++) {
                int tagId = (int) inputs[i].tagIDs[z];
                lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
                Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose((int) inputs[i].tagIDs[z]);
                tagPose.ifPresent(tagPoses::add);
            }

            // Gets the average distance to tag
            double avgDistance = inputs[i].avgTagDist;

            // Calculate standard deviation to give to the .addVisionData() swerve method
            // Standard Deviation is inverse to confidence level
            xyStdDev = VisionConstants.XY_STD_DEV_COEF * (avgDistance * avgDistance)
                    / (inputs[i].tagCount * inputs[i].tagCount);
            double thetaStdDev = VisionConstants.THETA_STD_DEV_COEF;

            // Add vision data to swerve pose estimator
            VisionData visionData = new VisionData(visionCalcPose, inputs[i].captureTimestamp,
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));
                visionDataConsumer.accept(visionData);

            // Add robot pose from this camera to a list of all robot poses
            allRobotPoses.add(visionCalcPose);
            // xyStdDev = 200;
        }
        visionPoseArrayPublisher.set(allRobotPoses.toArray(new Pose2d[0]));
    }
}