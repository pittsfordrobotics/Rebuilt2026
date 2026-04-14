// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction; //for sysid

import frc.robot.generated.TunerConstants;
import frc.robot.lib.util.AllianceFlipUtil;
import frc.robot.lib.util.ShooterHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIO.IMUMode;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.VisionConstants;

public class RobotContainer {
    private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

    @Logged(name="Vision")
    public final Vision vision;

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    private final SendableChooser<Command> autoChooser;

    private GenericEntry testingDistToHub;

    private final boolean shouldMirrorAutos = true;

    @Logged(name = "Swerve")
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(driverController);

    @Logged(name = "PDH")
    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    @Logged(name = "Intake")
    private final Intake intake;

    @Logged(name = "Indexer")
    private final Indexer indexer;

    @Logged(name = "Shooter")
    private final Shooter shooter;

    // @Logged(name = "Climber")
    // private final Climber climber;

    @Logged(name = "Hood")
    private final Hood hood;

    public RobotContainer() {
	    DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
		
        vision = new Vision(
            () -> drivetrain.getState().Pose.getRotation(),
            () -> drivetrain.getState().Speeds.omegaRadiansPerSecond,
            drivetrain::addVisionMeasurement,
            // VisionConstants.LIMELIGHT_LEFT,
            // VisionConstants.LIMELIGHT_RIGHT,
            VisionConstants.LIMELIGHT_FRONT);


        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        intake= new Intake();
        shooter = new Shooter();
        indexer = new Indexer();
        // climber = new Climber();
        hood = new Hood();

        NamedCommands.registerCommand("ShootatHub", autoDecideShooting());
        NamedCommands.registerCommand("IntakeOut", intake.pivotOut());
        NamedCommands.registerCommand("IntakeIn", intake.pivotIn());
        NamedCommands.registerCommand("IntakeRun", intake.runIntake());
        NamedCommands.registerCommand("HoodDown", hood.runHoodClose());


        new EventTrigger("IntakeOutEvent").onTrue(intake.pivotOut());
        new EventTrigger("RunIntakeEvent").whileTrue(intake.runIntake());
        new EventTrigger("HoodDown").whileTrue(hood.runHoodClose());


        autoChooser = shouldMirrorAutos ? AutoBuilder.buildAutoChooserWithOptionsModifier(this::mirrorAutos) : AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
        testingShuffleboardInit();
    }


    private void configureBindings() {
        drivetrain.setDefaultCommand(drivetrain.drive().withName("DefaultDrive"));
        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true).withName("Idle")
        );
        
        driverController.a().toggleOnTrue(drivetrain.brake().withName("Brake"));
        driverController.b().whileTrue(drivetrain.pointAtHub().withName("PointAtHub"));
        driverController.y().onTrue(Commands.runOnce(() -> drivetrain.enableSlowDrive()))
            .onFalse(Commands.runOnce(() -> drivetrain.disableSlowDrive()));
        driverController.x().onTrue(Commands.runOnce(() -> drivetrain.enableCircleDrive()))
            .onFalse(Commands.runOnce(() -> drivetrain.disableCircleDrive()));

        operatorController.rightBumper().whileTrue(shooter.runShooter().withName("ManualRunShooter"));
        operatorController.leftBumper().whileTrue(indexer.runIndex().withName("ManualRunIndex"));
        //Run Shooter
        operatorController.b().whileTrue(
            Commands.runOnce(() -> drivetrain.enableSlowDrive())
            .andThen(autoDecideShooting()).withName("AutoDecideShooting"))
            .onFalse(Commands.runOnce(() -> drivetrain.disableSlowDrive()).andThen(intake.pivotOut()));

        // operatorController.y().whileTrue(
        //     Commands.runOnce(() -> drivetrain.enableCircleDrive())
        //     .andThen(shootWhileMove()))
        //     .onFalse(Commands.runOnce(() -> drivetrain.disableCircleDrive())
        //     .andThen(intake.pivotOut()));
        operatorController.y().whileTrue(shootWhileMove().withName("shootWhileMove"));
        
        // operatorController.y().whileTrue(climbUp());
        // operatorController.y().whileFalse(climber.runClimber(() -> -0.05));
        // operatorController.x().whileTrue(climbDown());
        operatorController.a().whileTrue(intake.pivotOut().andThen(intake.runIntake()).withName("RunIntake"));
        operatorController.povLeft().onTrue(intake.pivotIn().withName("PivotIn"));
        operatorController.leftTrigger().whileTrue(intake.agitate().withName("ManualAgitate"));
        // operatorController.y().whileTrue(Commands.parallel(
        //             hood.runHood(() -> 0.4),
        //             shooter.runShooter(() -> 0.9, () -> 0).until(() -> shooter.isAtSpeed()).andThen(shooter.runShooter(() -> 0.9, () -> 0.7)), 
        //             indexer.runIndex(),
        //             intake.agitate(),
        //             drivetrain.pointAtAllianceZone()));

        operatorController.povRight().onTrue(intake.resetEncoder().ignoringDisable(true).withName("ResetEncoder"));

        operatorController.povUp().onTrue(hood.runHood(() -> HoodConstants.FAR_SHOOTING_SETPOINT));
        operatorController.povDown().onTrue(hood.runHood(() -> HoodConstants.CLOSE_SHOOTING_SETPOINT));

        operatorController.back().onTrue(intake.resetEncoderOut().ignoringDisable(true));

        operatorController.x().whileTrue(trenchShoot().withName("TrenchShoot"))
            .onFalse(intake.pivotOut());
        operatorController.rightTrigger().whileTrue(Commands.parallel(intake.extake(), indexer.runIndex(() -> -0.6)));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverController.leftBumper().onTrue(
                Commands.runOnce(() -> { vision.setIMU(IMUMode.EXTERNAL_SEED.getNum()); }).ignoringDisable(true)
                    .andThen(() -> { drivetrain.resetRotation(AllianceFlipUtil.isRed() ? Rotation2d.k180deg : Rotation2d.kZero); }).ignoringDisable(true))
            .onFalse(Commands.runOnce(() ->{ vision.setIMU(IMUMode.INTERNAL_EXTERNAL_ASSIST.getNum()); }).ignoringDisable(true));

        drivetrain.registerTelemetry(logger::telemeterize);
    } 

    private void testingShuffleboardInit(){
        testingDistToHub = Shuffleboard.getTab("testing").add("Testing Dist to Hub", 100).getEntry();
        Shuffleboard.getTab("testing").add("Drive to testing point", drivetrain.driveToDistFromBlueHub(() -> testingDistToHub.getDouble(0)));
    }
    

    public Command getAutonomousCommand() {
        try {
            return autoChooser.getSelected();
        } catch(Exception e) {
            System.out.println(e.toString());
            return null;
        }
    }

    // public Command climbDown() {
    //    return climber.runClimber(() -> 0.4);
    // }

    // public Command climbUp(){
    //     return drivetrain.driveToPose(ClimberConstants.FLIPPED_CLIMB_UNEXTENDED_POS)
    //         .alongWith(climber.runClimber(() -> 0), intake.pivotIn())
    //         .andThen(drivetrain.driveToPose(ClimberConstants.FLIPPED_CLIMB_EXTENDED_POS))
    //         .andThen(climber.runClimber(() -> -0.4));
    // }

    private Command shootWhileMove() {
        // return autoDecideShooting(false);

        return Commands.parallel(
                hood.runHoodForShoot(() -> drivetrain.getState().Pose),
                Commands.waitSeconds(0.0) // Wait in case the hood needs to change position.
                    .andThen(shooter.shootAtHub(() -> drivetrain.getState().Pose, () -> false)
                        .until(() -> shooter.isAtSpeed()))
                        .andThen(shooter.shootAtHub(() -> drivetrain.getState().Pose, () -> true)),
                indexer.runIndex(),
                intake.agitate());
    }

    private Command autoDecideShooting(boolean brake) {
        return Commands.defer(() -> {
            if (ShooterHelpers.isPassing(() -> drivetrain.getState().Pose)){
                // In neutral zone, set the shooter to pass to the alliance area.
                return Commands.parallel(
                    hood.runHood(() -> HoodConstants.PASSING_SETPOINT),
                    shooter.runShooter(() -> 0.9, () -> 0).until(() -> shooter.isAtSpeed()).andThen(shooter.runShooter(() -> 0.9, () -> 0.7)), 
                    indexer.runIndex(),
                    intake.agitate(),
                    drivetrain.pointAtAllianceZone());
            }
            // In the alliance area, set the shooter to shoot in the hub.
            return Commands.parallel(
                hood.runHoodForShoot(() -> drivetrain.getState().Pose),
                Commands.waitSeconds(0.0) // Wait in case the hood needs to change position.
                    .andThen(shooter.shootAtHub(() -> drivetrain.getState().Pose, () -> false)
                        .until(() -> (shooter.isAtSpeed() && hood.isAtSetpoint())))
                        .andThen(shooter.shootAtHub(() -> drivetrain.getState().Pose, () -> true)),
                indexer.runIndex(),
                intake.agitate(),
                brake ? drivetrain.pointAtHubStatic() : drivetrain.pointAtHub());
            // return Commands.parallel(
            //     hood.runHoodForShoot(() -> drivetrain.getState().Pose),
            //     shooter.shootAtHub(() -> drivetrain.getState().Pose, () -> false)
            //             .until(() -> shooter.isAtSpeed()))
            //             .andThen(shooter.shootAtHub(() -> drivetrain.getState().Pose, () -> true),
            //     indexer.runIndex(),
            //     intake.agitate(),
            //     drivetrain.pointAtHub());
        }, Set.of(shooter, indexer, drivetrain));
    }

    public Command autoDecideShooting() {
        return autoDecideShooting(true);
    }

    public Command trenchShoot() {
        //This method is for in case vision/odometry is catastrophically broken and we need to shoot regardless. This has constants
        //for a fixed position and will shoot reliably from there
        return Commands.parallel(
                hood.runHood(() -> 0.35),
                indexer.runIndex(),
                intake.agitate(),
                Commands.waitSeconds(0.3) // Wait in case the hood needs to change position.
                    .andThen(shooter.trenchShoot()));
    }

    private Stream<PathPlannerAuto> mirrorAutos(Stream<PathPlannerAuto> autos)
    {
        List<PathPlannerAuto> newList = new ArrayList<>();
        List<PathPlannerAuto> existingList = autos.toList();
        for (PathPlannerAuto auto : existingList)
        {
            newList.add(auto);
            String autoName = auto.getName();
            PathPlannerAuto mirroredAuto = new PathPlannerAuto(autoName, true);
            mirroredAuto.setName(autoName + " (Mirrored)");
            newList.add(mirroredAuto);
        }
        
        return newList.stream();
    }
}
