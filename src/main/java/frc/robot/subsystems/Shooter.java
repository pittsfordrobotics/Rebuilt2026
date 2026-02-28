// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.lib.util.ShooterHelpers;

import static edu.wpi.first.units.Units.*;

public class Shooter extends SubsystemBase {
	/** Creates a new Shooter. */

	private GenericEntry shooterSpeed;
	private GenericEntry uptakeSpeed;
	private double currentSetSpeed;
	
	private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

	@Logged(name="Is at speed")
	public boolean isAtSpeed() {
		if (currentSetSpeed == 0) {
			return false;
		}
		if (this.getMiddleMotor().getVelocity().getValue().in(RPM) >= (currentSetSpeed*(ShooterConstants.kFreeSpeed.in(RotationsPerSecond)*60)) * ShooterConstants.IS_AT_SPEED_PERCENTAGE) {
			return true;
		} else {
			return false;
		}
	}

	public final TalonFX[] shooterMotors = new TalonFX[ShooterConstants.SHOOTER_MOTORS.length];

	@Logged(name="Uptake Motor")
	final TalonFX uptakeMotor = new TalonFX(ShooterConstants.UPTAKE_MOTOR);

	

	public Shooter() {

		TalonFXConfiguration shooterConfig = new TalonFXConfiguration()
				.withMotorOutput(new MotorOutputConfigs()
					.withNeutralMode(NeutralModeValue.Coast))
				.withVoltage(
					new VoltageConfigs()
						.withPeakReverseVoltage(Volts.of(0))
				)
				.withCurrentLimits(new CurrentLimitsConfigs()
					.withStatorCurrentLimit(Amps.of(120))
					.withStatorCurrentLimitEnable(true)
					.withSupplyCurrentLimit(Amps.of(70))
					.withSupplyCurrentLimitEnable(true))
				.withSlot0(
					new Slot0Configs()
					.withKP(0.5)
					.withKI(2)
					.withKD(0)
					.withKV(12.0 / ShooterConstants.kFreeSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
				);

		TalonFXConfiguration uptakeConfig = new TalonFXConfiguration()
				.withMotorOutput(new MotorOutputConfigs()
					.withNeutralMode(NeutralModeValue.Coast))
				.withCurrentLimits(new CurrentLimitsConfigs()
					.withStatorCurrentLimit(Amps.of(120))
					.withStatorCurrentLimitEnable(true))
				.withMotorOutput(new MotorOutputConfigs()
					.withInverted(InvertedValue.Clockwise_Positive));

		// instantiate motor controllers
		for (int i = 0; i < shooterMotors.length; i++) {
			shooterMotors[i] = new TalonFX(ShooterConstants.SHOOTER_MOTORS[i]);
			shooterMotors[i].getConfigurator().apply(shooterConfig);
		}

		uptakeMotor.getConfigurator().apply(uptakeConfig);
		Shuffleboard.getTab("testing").add("Run Shooter", this.runShooter(null, null));

		shooterSpeed = Shuffleboard.getTab("testing").add("Shooter Motor Speed", .6).getEntry();
		uptakeSpeed = Shuffleboard.getTab("testing").add("Uptake Motor Speed", .6).getEntry();
		// Shuffleboard.getTab("testing").add("Shoot at Hub", this.)
	}

	public Command runShooter(DoubleSupplier shootSpeed, DoubleSupplier uptakeSpeed) {
		return run(() -> {
			currentSetSpeed = shootSpeed.getAsDouble();
			for (TalonFX motor : shooterMotors) {
				motor.setControl(velocityRequest.withVelocity(RPM.of(shootSpeed.getAsDouble()*(ShooterConstants.kFreeSpeed.in(RotationsPerSecond)*60))));
			}
			uptakeMotor.set(uptakeSpeed.getAsDouble());
		}).finallyDo(() -> {
			currentSetSpeed = 0;
			for (TalonFX motor : shooterMotors) {
				motor.set(0);
			}
			uptakeMotor.set(0);
		});
	}

	public Command runShooter() {
		if (isAtSpeed()){
			return runShooter(() -> shooterSpeed.getDouble(shooterSpeed.getDouble(currentSetSpeed)), 
		() -> uptakeSpeed.getDouble(ShooterConstants.UPTAKE_SPEED));
		} else {
			return runShooter(() -> shooterSpeed.getDouble(shooterSpeed.getDouble(currentSetSpeed)), 
		() -> uptakeSpeed.getDouble(0));
		}
        
    }

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

    @Logged(name="Middle Motor")
    public TalonFX getMiddleMotor() {
        return this.shooterMotors[1];
    }

	@Logged(name="Left Motor")
    public TalonFX getLeftMotor() {
        return this.shooterMotors[0];
    }
	
	@Logged(name="Right Motor")
    public TalonFX getRightMotor() {
        return this.shooterMotors[2];
    }

	public Command shootAtHub(Supplier<Pose2d> currentPose, Supplier<Boolean> runUptake) {
		return runShooter(() -> shootHubSpeed(currentPose), () -> {return runUptake.get() ? .6 : 0;});
	}

	public Command shootAtHub(Supplier<Pose2d> currentPose) {
		return runShooter(() -> shootHubSpeed(currentPose), () -> .6 );
	}

	public double shootHubSpeed(Supplier<Pose2d> currentPose) {
		double hubDist = ShooterHelpers.getHubDistInches(currentPose);
		// System.out.println("\n\n\n" + hubDist + "\n\n\n");
		if(hubDist < 100) {
			return 0.0022*hubDist + 0.2932;
		} else {
			return 0.0019*hubDist + 0.2718;
		}
	}

	public Command startShooter(Object object) {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'startShooter'");
	}
}
