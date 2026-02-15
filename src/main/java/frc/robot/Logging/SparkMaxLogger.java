package frc.robot.Logging;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(SparkMax.class)
public class SparkMaxLogger extends ClassSpecificLogger<SparkMax> {
    public SparkMaxLogger() {
        super(SparkMax.class);
    }

    @Override
    public void update(EpilogueBackend backend, SparkMax sparkMax) {
        backend.log("SparkMax ID", sparkMax.getDeviceId());
        backend.log("Motor Temperature (C)", sparkMax.getMotorTemperature());
        backend.log("Output Current", sparkMax.getOutputCurrent());
        backend.log("Absolute Encoder Position", sparkMax.getAbsoluteEncoder().getPosition());
        backend.log("Absolute Encoder Velocity", sparkMax.getAbsoluteEncoder().getVelocity());
        backend.log("Encoder Position", sparkMax.getEncoder().getPosition());
        backend.log("Encoder Velocity", sparkMax.getEncoder().getVelocity());
        backend.log("Applied Output", sparkMax.getAppliedOutput());
    }

}
