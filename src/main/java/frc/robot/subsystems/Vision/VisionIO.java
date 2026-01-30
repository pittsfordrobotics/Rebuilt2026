package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;

/** Vision subsystem hardware interface. */
public interface VisionIO {
    class VisionIOInputs {
        public double totalLatency = 0.0;
        public double captureTimestamp = 0.0;
        public boolean hasTarget = false;
        public boolean connected = false;
        public double avgTagDist = 0.0;
        public int tagCount = 0;
        public double[] tagDistances = new double[]{};
        public int[] tagIDs = new int[]{};
        public Pose2d pose = new Pose2d();
    }

    /**
     * Far: has the lowest FPS, but allows for possible vision updates when halfway down the field
     * Mid: has slightly higher FPS, but allows for farther vision updates when halfway down the field
     * Close: has the highest FPS, but only has limited range
     */
    
    enum Pipelines {
        Test(0);

        private final int num;
        Pipelines(int num) {
            this.num = num;
        }

        public int getNum() {
            return num;
        }
    }

    enum LED {
        PIPELINE(0), OFF(1), BLINK(2), ON(3);

        private final int num;

        LED(int num) {
            this.num = num;
        }

        public int getNum() {
            return num;
        }
    }

    enum CameraMode {
        VISION_PROCESSING(0), DRIVER_CAMERA(1);

        private final int num;

        CameraMode(int num) {
            this.num = num;
        }

        public int getNum() {
            return num;
        }
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(VisionIOInputs inputs, double gyroAngle) {}

    /** Enabled or disabled vision LEDs. */
    default void setCameraModes(CameraMode mode) {}

    /** Sets the pipeline number. */
    default void setPipeline(Pipelines pipeline) {}

}