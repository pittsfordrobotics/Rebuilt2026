package frc.robot.Logging;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.units.Units;

@CustomLoggerFor(TalonFX.class)
public class KrakenLogger extends ClassSpecificLogger<TalonFX> {
    public KrakenLogger() {
        super(TalonFX.class);
    }

    @Override
    public void update(EpilogueBackend backend, TalonFX talonFX) {
        //Log motor IDs
        backend.log("Motor ID", talonFX.getDeviceID());
        backend.log("Temperature (C)", talonFX.getDeviceTemp().getValue().in(Units.Celsius));
        backend.log("Voltage (Volts)", talonFX.getMotorVoltage().getValue().in(Units.Volts));
        backend.log("Current (Amps)", talonFX.getSupplyCurrent().getValue().in(Units.Amps));
        backend.log("Position Revolutions", talonFX.getPosition().getValue().in(Units.Revolutions));
        backend.log("Velocity RPM", talonFX.getVelocity().getValue().in(Units.RPM));
    }
    
}
