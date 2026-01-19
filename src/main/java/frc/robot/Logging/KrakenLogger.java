package frc.robot.Logging;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(SwerveModule.class)
public class KrakenLogger extends ClassSpecificLogger<SwerveModule> {
    public KrakenLogger() {
        super(SwerveModule.class);
    }

    @Override
    public void update(EpilogueBackend backend, SwerveModule swerveModule) {
        //Log motor IDs
        backend.log("Drive Motor ID", swerveModule.getDriveMotor().getDeviceID());
        backend.log("Swerve Motor ID", swerveModule.getSteerMotor().getDeviceID());
    }
    
}
