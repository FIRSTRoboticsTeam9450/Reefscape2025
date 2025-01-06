// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntakeSubsystem;

public class RobotContainer {
    private double MaxSpeed = 0.0;//TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = 0.0;//RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driver1 = new CommandXboxController(0);
    private final CommandXboxController m_driver2 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driver2.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driver2.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_driver2.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        m_driver2.a().whileTrue(drivetrain.applyRequest(() -> brake));
        m_driver2.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driver2.getLeftY(), -m_driver2.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driver2.back().and(m_driver2.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driver2.back().and(m_driver2.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driver2.start().and(m_driver2.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driver2.start().and(m_driver2.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        m_driver2.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        /* ----- Coral Intake System ----- */

        CoralIntakeSubsystem CI = CoralIntakeSubsystem.getInstance();

        /*
         * A = Rotate down
         * Y = Rotate up
         * X = Tilt Left
         * B = Tilt Right
         */

         double voltage = 0.175;
        //On trues
        m_driver1.a().onTrue(new CoralIntakeCommand(voltage, voltage));
        m_driver1.y().onTrue(new CoralIntakeCommand(-voltage, -voltage));
        m_driver1.x().onTrue(new CoralIntakeCommand(voltage, -voltage));
        m_driver1.b().onTrue(new CoralIntakeCommand(-voltage, voltage));
        // m_driver1.a().onTrue(new CoralIntakeCommand(CI.voltage, CI.voltage));
        // m_driver1.y().onTrue(new CoralIntakeCommand(-CI.voltage, -CI.voltage));
        // m_driver1.x().onTrue(new CoralIntakeCommand(CI.voltage, -CI.voltage));
        // m_driver1.b().onTrue(new CoralIntakeCommand(-CI.voltage, CI.voltage));

        //On Falses
        m_driver1.a().onFalse(new CoralIntakeCommand(0, 0));
        m_driver1.y().onFalse(new CoralIntakeCommand(0, 0));
        m_driver1.x().onFalse(new CoralIntakeCommand(0, 0));
        m_driver1.b().onFalse(new CoralIntakeCommand(0, 0));

        //Coral Intake PID Keybinds
        // m_driver1.a().onTrue(new CoralIntakeCommand(.3, true));
        // m_driver1.y().onTrue(new CoralIntakeCommand(.6, true));
        // m_driver1.x().onTrue(new CoralIntakeCommand(.7, false));
        // m_driver1.b().onTrue(new CoralIntakeCommand(.55, false));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
