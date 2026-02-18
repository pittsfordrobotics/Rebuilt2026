// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction; //for sysid

import frc.robot.generated.TunerConstants;
import frc.robot.lib.util.AllianceFlipUtil;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import frc.robot.subsystems.Vision.Vision;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;

public class RobotContainer {
    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    public final Vision vision;

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final SendableChooser<Command> autoChooser;


    @Logged(name = "Swerve")
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(driverController);

    @Logged(name = "PDH")
    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    private final Intake intake;
    private final Indexer indexer;

    @Logged(name="Shooter")
    private final Shooter shooter;
    private final Climber climber;

    public RobotContainer() {
	    DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
		
        vision = new Vision(
            () -> drivetrain.getState().RawHeading,
            () -> drivetrain.getState().Speeds.omegaRadiansPerSecond,
            drivetrain::addVisionMeasurement,
            VisionConstants.LIMELIGHT_LEFT,
            VisionConstants.LIMELIGHT_RIGHT);

        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
        intake= new Intake();
        shooter = new Shooter();
        indexer = new Indexer();
        climber = new Climber();

        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.drive());
        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        driverController.a().toggleOnTrue(drivetrain.brake());
        driverController.b().whileTrue(drivetrain.pointAtHub());
        driverController.x().whileTrue(drivetrain.driveToPoint(FieldConstants.flippedHubPosition));
        driverController.x().and(driverController.b()).whileTrue(drivetrain.driveToAndPointAt(FieldConstants.flippedHubPosition));

        operatorController.rightBumper().whileTrue(shooter.runShooter());
        operatorController.leftBumper().whileTrue(indexer.runIndex());
        operatorController.b().whileTrue(Commands.parallel(shooter.runShooter(), indexer.runIndex(), drivetrain.pointAtHub()));
        operatorController.y().whileTrue(climbUp());
        operatorController.x().whileTrue(climbDown());
        operatorController.a().onTrue(intake.pivotOut().andThen(intake.runIntake(() -> .1)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(drivetrain.runOnce(
            () -> drivetrain.resetRotation(AllianceFlipUtil.isRed() ? Rotation2d.k180deg : Rotation2d.kZero)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }
    

    public Command getAutonomousCommand() {
        try {
            return autoChooser.getSelected();
        } catch(Exception e) {
            System.out.println(e.toString());
            return null;
        }
    }

    public Command pointAt(Supplier<Translation2d> targetPoint) {
        return drivetrain.applyRequest(() -> {
                Translation2d currentPoint = drivetrain.getState().Pose.getTranslation();
                Rotation2d targetHeading = SwerveHelpers.getAngleToPoint(currentPoint, targetPoint.get());
                double[] leftDeadbanded = SwerveHelpers.swerveDeadband(new double[]{joystick.getLeftX(), joystick.getLeftY()}, .1);
                return driveHeading.withVelocityX(leftDeadbanded[1] * MaxSpeed)
                    .withVelocityY(leftDeadbanded[0] * MaxSpeed)
                    .withTargetDirection(targetHeading);
        });
    }

    public Command pointAtHub() {
        return pointAt(FieldConstants.flippedHubPosition);
    }

    public Command driveToPose(Supplier<Pose2d> targetPose) {
        PathConstraints constraints = PathConstraints.unlimitedConstraints(12);
        return Commands.defer(() -> AutoBuilder.pathfindToPose(targetPose.get(), constraints), Set.of(drivetrain));
    }

    public Command driveToPoint(Supplier<Translation2d> targetPoint) {
        return driveToPose(() -> new Pose2d(targetPoint.get(), drivetrain.getState().Pose.getRotation()));
    }

    public Command driveToAndPointAt(Supplier<Translation2d> targetPoint) {
        return driveToPose(() -> {
            Translation2d currentPoint = drivetrain.getState().Pose.getTranslation();
            Rotation2d targetHeading = SwerveHelpers.getAngleToPoint(currentPoint, targetPoint.get());
            return new Pose2d(targetPoint.get(), targetHeading);
        });
    }
}
