package frc.robot.lib.util;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Frequency;

public class TalonConfigurator {
    private static final Frequency onePerSecond = Frequency.ofRelativeUnits(1, Units.Hertz);
    private static final Frequency fourPerSecond = Frequency.ofRelativeUnits(4, Units.Hertz);
    private static final Frequency fiftyPerSecond = Frequency.ofRelativeUnits(50, Units.Hertz);

    public static void ReduceCommonStatusFrameFrequencies(TalonFX talonFx)
    {
        // Version status frames default to 4 Hz.
        talonFx.getVersion().setUpdateFrequency(onePerSecond);
        talonFx.getVersionMajor().setUpdateFrequency(onePerSecond);
        talonFx.getVersionMinor().setUpdateFrequency(onePerSecond);
        talonFx.getVersionBugfix().setUpdateFrequency(onePerSecond);

        // These default to 100 Hz.
        talonFx.getMotorVoltage().setUpdateFrequency(fourPerSecond);
        talonFx.getForwardLimit().setUpdateFrequency(fourPerSecond);
        talonFx.getReverseLimit().setUpdateFrequency(fourPerSecond);
        talonFx.getDutyCycle().setUpdateFrequency(fourPerSecond);
        talonFx.getTorqueCurrent().setUpdateFrequency(fourPerSecond);
        
        // These default to 100 Hz.
        // Keeping these relatively high, as they seem to be used for swerve feedback.
        talonFx.getAppliedRotorPolarity().setUpdateFrequency(fiftyPerSecond);
        talonFx.getPosition().setUpdateFrequency(fiftyPerSecond);
        talonFx.getVelocity().setUpdateFrequency(fiftyPerSecond);
        talonFx.getAcceleration().setUpdateFrequency(fiftyPerSecond);
    }
}
