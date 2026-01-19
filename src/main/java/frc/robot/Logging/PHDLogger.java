package frc.robot.Logging;

import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.CustomLoggerFor;

import edu.wpi.first.wpilibj.PowerDistribution;

@CustomLoggerFor(PowerDistribution.class)
public class PHDLogger extends ClassSpecificLogger<PowerDistribution>{
    public PHDLogger () {
        super(PowerDistribution.class);
    }

    @Override
    public void update(EpilogueBackend backend, PowerDistribution pdh) {
        backend.log("PDH Voltage (Volts)", pdh.getVoltage());
        backend.log("PDH Current (Amps)", pdh.getTotalCurrent());
        backend.log("PDH Temperature (Degrees Celsius)", pdh.getTemperature());
        backend.log("All PDH Currents", pdh.getAllCurrents());
    }
}
