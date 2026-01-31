package frc.robot.Logging;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TalonFX.class)
public class KrakenLogger extends ClassSpecificLogger<TalonFX> {
    public KrakenLogger() {
        super(TalonFX.class);
    }

    @Override
    public void update(EpilogueBackend backend, TalonFX talonFX) {
        //Log motor IDs
        backend.log("Motor ID", talonFX.getDeviceID());
        backend.log("Temperature", talonFX.getDeviceTemp().getValue());
        backend.log("Voltage", talonFX.getMotorVoltage().getValue());
        backend.log("Position", talonFX.getPosition().getValue());
        backend.log("Velocity", talonFX.getVelocity().getValue());
        
    }
    
}
