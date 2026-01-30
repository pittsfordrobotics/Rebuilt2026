// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction; //for sysid

import frc.robot.generated.TunerConstants;
import frc.robot.lib.VisionData;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIOLimelight;
import frc.robot.constants.VisionConstants;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(2.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final FieldCentric drive = new FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final FieldCentricFacingAngle driveHeading = new FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withHeadingPID(10, 0, 0);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final Vision vision;

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Intake intake;
    private final Indexer indexer;
    private final Shooter shooter;
    private GenericEntry intakeSpeed;
    private GenericEntry shooterSpeed;
    private GenericEntry indexSpeed;

    public RobotContainer() {
        vision = new Vision(
            VisionConstants.LIMELIGHT_LEFT,
            VisionConstants.LIMELIGHT_RIGHT, 
            () -> drivetrain.getState().RawHeading,
            () -> drivetrain.getState().Speeds.omegaRadiansPerSecond,
            drivetrain::addVisionMeasurement);

        configureBindings();
        intake= new Intake();
        shooter = new Shooter();
        indexer = new Indexer();



        intakeSpeed = Shuffleboard.getTab("testing").add("Intake Motor Speed", .25).getEntry();
        Shuffleboard.getTab("testing").add("Run Intake", intake.runIntake(() -> intakeSpeed.getDouble(0.25)));



        indexSpeed = Shuffleboard.getTab("testing").add("Intake Mover Speed", .25).getEntry();
        Shuffleboard.getTab("testing").add("Run Indexer", indexer.runIndex(() -> indexSpeed.getDouble(0.25)));


        shooterSpeed = Shuffleboard.getTab("testing").add("Shooter Motor Speed", .25).getEntry();
        Shuffleboard.getTab("testing").add("Run Shooter", shooter.runShooter(() -> shooterSpeed.getDouble(0.25)));
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                double[] leftDeadbanded = AllDeadbands.applyScalingCircularDeadband(new double[]{joystick.getLeftX(), joystick.getLeftY()}, .1);
                Rotation2d heading = getHeadingFromStick(() -> -joystick.getRightY(), () -> -joystick.getRightX());
                if(heading != null) {
                    return driveHeading.withVelocityX(leftDeadbanded[1] * MaxSpeed)
                        .withVelocityY(leftDeadbanded[0] * MaxSpeed)
                        .withTargetDirection(heading);
                }

                return drive.withVelocityX(leftDeadbanded[1] * MaxSpeed)
                    .withVelocityY(leftDeadbanded[0] * MaxSpeed)
                    .withRotationalRate((joystick.getLeftTriggerAxis() - joystick.getRightTriggerAxis()) * MaxAngularRate);
            })
                
        );
        

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        
        joystick.a().toggleOnTrue(drivetrain.applyRequest(() -> brake));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        
    }

    public Rotation2d getHeadingFromStick(DoubleSupplier rotationX, DoubleSupplier rotationY) {
        Rotation2d heading;
        double[] deadbandRotationInputs = AllDeadbands
                .applyScalingCircularDeadband(new double[] { rotationX.getAsDouble(), rotationY.getAsDouble() }, 0.95);
        if (deadbandRotationInputs[0] != 0 || deadbandRotationInputs[1] != 0) {
            heading = Rotation2d.fromRadians(Math.atan2(deadbandRotationInputs[1], deadbandRotationInputs[0]));
        } else {
            heading = null; //keep existing heading
        }
        return heading;
    }

    public Command getAutonomousCommand() {
        try{
        return new PathPlannerAuto("Test Auto");
        }
        catch(Exception e){
            System.out.println(e.toString());
            return null;
        }
    }
}
