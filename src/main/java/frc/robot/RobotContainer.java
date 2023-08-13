// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.IntakeandWristConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(AutoConstants.kS, AutoConstants.kV, AutoConstants.kA);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(
                                        Math.pow(m_driverController.getLeftY(), 2)
                                                * Math.signum(m_driverController.getLeftY()),
                                        OIConstants.kDriveDeadband), // squaring inputs to make robot more controllable
                                -MathUtil.applyDeadband(
                                        Math.pow(m_driverController.getLeftX(), 2)
                                                * Math.signum(m_driverController.getLeftX()),
                                        OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(),
                                        OIConstants.kDriveDeadband),
                                true, true),
                        m_robotDrive));
    }
    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(m_driverController, 1)
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.setX(),
                        m_robotDrive));

        new JoystickButton(m_driverController, 2)
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.zeroHeading(),
                        m_robotDrive));

        new JoystickButton(m_driverController, 6)
                .whileTrue(
                        // The left stick controls translation of the robot.
                        // Turning is controlled by the X axis of the right stick.

                        //robot relative
                        new RunCommand(
                                () -> m_robotDrive.drive(
                                        -MathUtil.applyDeadband(
                                                Math.pow(m_driverController.getLeftY(), 2)
                                                        * Math.signum(m_driverController.getLeftY()),
                                                OIConstants.kDriveDeadband), // squaring inputs to make robot more
                                                                             // controllable
                                        -MathUtil.applyDeadband(
                                                Math.pow(m_driverController.getLeftX(), 2)
                                                        * Math.signum(m_driverController.getLeftX()),
                                                OIConstants.kDriveDeadband),
                                        -MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(),
                                                OIConstants.kDriveDeadband),
                                        false, true),
                                m_robotDrive));

        new JoystickButton(m_driverController, 7)
                .whileTrue(
                        // The left stick controls translation of the robot.
                        // Turning is controlled by the X axis of the right stick.

                        //slow mode

                        //not tested slow mode

                        new RunCommand(
                                () -> m_robotDrive.drive(
                                        -MathUtil.applyDeadband(
                                                (m_driverController.getLeftY() / 2),
                                                OIConstants.kDriveDeadband), 

                                        -MathUtil.applyDeadband(
                                                (m_driverController.getLeftX() / 2),
                                                OIConstants.kDriveDeadband),

                                        -MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(),
                                                OIConstants.kDriveDeadband),
                                        true, true),
                                m_robotDrive));


                                //intake in and wrist down
        new JoystickButton(m_driverController, 8)
                .whileTrue(
                        new InstantCommand(() -> Intake.intakeMotor.set(1), 
                        Intake.m_wristPIDController.setReference(IntakeandWristConstants.kMinAngle, CANSparkMax.ControlType.kPosition))
                );
        new JoystickButton(m_driverController, 8)
                .whileFalse(
                        new InstantCommand(() -> Intake.intakeMotor.set(0.1),
                        Intake.m_wristPIDController.setReference(IntakeandWristConstants.kMaxAngle, CANSparkMax.ControlType.kPosition))
                );

                //intake out
        new JoystickButton(m_driverController, 9)
                .whileTrue(
                        new InstantCommand(() -> Intake.intakeMotor.set(-1))
                );
        new JoystickButton(m_driverController, 9)
                .whileFalse(
                        new InstantCommand(() -> Intake.intakeMotor.set(0))
                );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3,0, new Rotation2d(90)),
        config);



        Trajectory CubeDrop = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0.2, 0), new Translation2d(0, 0), new Translation2d(2, 0)),
                // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(4,0, new Rotation2d(0)),
        config);


        Trajectory oneMeterTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1,0, new Rotation2d(0)),
            config);

        var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0,
        AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new
        SwerveControllerCommand(
        CubeDrop,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

        // // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // This will load the file "Example Path.path" and generate it with a max
        // velocity of 4 m/s and a max acceleration of 3 m/s^2
        // // Run path following command, then stop at the end
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, true, false));
        // }
    }
}
