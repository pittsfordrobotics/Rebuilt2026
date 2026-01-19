package frc.robot.Logging;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

import edu.wpi.first.wpilibj.DriverStation;

@CustomLoggerFor(DriverStation.class)
public class DriveStationLogger extends ClassSpecificLogger<DriverStation> {
    public DriveStationLogger() {
        super(DriverStation.class);
    }

    @Override
    public void update(EpilogueBackend backend, DriverStation driverStation) {
        

    }
}
