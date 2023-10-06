// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.time.Instant;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
//import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeandWristConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems
    public final DriveSubsystem m_robotDrive = new DriveSubsystem();
    public Intake intake = new Intake();
    public Wrist wrist = new Wrist();

    // The driver's controller
    public XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    private final CommandJoystick operator = new CommandJoystick(OIConstants.kOperatorControllerPort);
    //private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(AutoConstants.kS, AutoConstants.kV, AutoConstants.kA);
    String trajectoryJSON = "paths/output/TestPathweaver.wpilib.json";
    Trajectory testtrajectory = new Trajectory();
    //Command for autos
    SendableChooser<Command> autoChooser = new SendableChooser<>();

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
                                        m_driverController.getLeftY(),
                                        OIConstants.kDriveDeadband), // squaring inputs to make robot more controllable
                                -MathUtil.applyDeadband(
                                        m_driverController.getLeftX(),
                                        OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(),
                                        OIConstants.kDriveDeadband),
                                true, true),
                        m_robotDrive));

        wrist.setDefaultCommand(
                wrist.setWristTarget(IntakeandWristConstants.kStowAngle));

        intake.setDefaultCommand(
                new RunCommand( () -> Intake.intakeMotor.set(IntakeandWristConstants.kIntakeIdleSpeed),
                intake));

        try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                testtrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
              } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
              }
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
                //X button
        new JoystickButton(m_driverController, 1)
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.setX(),
                        m_robotDrive));

                        //back button
        new JoystickButton(m_driverController, 10)
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.zeroHeading(),
                        m_robotDrive));

                        //right trigger, robot cent
        // new JoystickButton(m_driverController, 8)
        //         .whileTrue(
        //                 // The left stick controls translation of the robot.
        //                 // Turning is controlled by the X axis of the right stick.

        //                 //robot relative
        //                 new RunCommand(
        //                         () -> m_robotDrive.drive(
        //                                 -MathUtil.applyDeadband(
        //                                         Math.pow(m_driverController.getLeftY(), 2)
        //                                                 * Math.signum(m_driverController.getLeftY()),
        //                                         OIConstants.kDriveDeadband), // squaring inputs to make robot more
        //                                                                      // controllable
        //                                 -MathUtil.applyDeadband(
        //                                         Math.pow(m_driverController.getLeftX(), 2)
        //                                                 * Math.signum(m_driverController.getLeftX()),
        //                                         OIConstants.kDriveDeadband),
        //                                 -MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(),
        //                                         OIConstants.kDriveDeadband),
        //                                 false, true),
        //                         m_robotDrive));

                                //left trigger
        new JoystickButton(m_driverController,3)
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


                                //intake in and wrist down         left bumper
        //new JoystickButton(m_driverController, 5)
        operator.button(1)
                .whileTrue(
                        new ParallelCommandGroup(wrist.setWristTarget(IntakeandWristConstants.kDeployAngle), intake.IntakeIn())
                );

        //new JoystickButton(m_driverController, 7)
        operator.button(3)
                .onTrue(
                        new ParallelCommandGroup(wrist.setWristTarget(IntakeandWristConstants.kStowAngle), intake.IntakeIdle())
                );

                //shoot cube      right bumper
        //new JoystickButton(m_driverController, 6)
        operator.button(2)
                .whileTrue(
                        new ParallelCommandGroup(wrist.setWristTarget(IntakeandWristConstants.kShootAngle), intake.getShootCubeCommand(wrist))
                );
    }

    public Command configureAutos() {
        m_robotDrive.m_gyro.reset();
        m_robotDrive.m_gyro.setAngleAdjustment(180);
        return new InstantCommand()
        .andThen(
                new ParallelCommandGroup(wrist.setWristTarget(IntakeandWristConstants.kDeployAngle), intake.IntakeIdle()).withTimeout(1)
        )
        .andThen(
                new ParallelCommandGroup(wrist.setWristTarget(IntakeandWristConstants.kDeployAngle), intake.IntakeIn()).withTimeout(2)
        )
        .andThen(
                new RunCommand(()->m_robotDrive.drive(0.1, 0, 0, true, false), m_robotDrive).withTimeout(3)
        )
        .andThen(
                new ParallelCommandGroup(wrist.setWristTarget(IntakeandWristConstants.kDeployAngle), intake.IntakeIn()).withTimeout(2)
        )
        //go backwards
        .andThen(new InstantCommand(()->{}))
        ;




    }



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command oldconfigautos() {
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
        new Pose2d(3,0, new Rotation2d(0)),
        config);


        Trajectory CubeDrop = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(0.2, 0), new Translation2d(0, 0), new Translation2d(2, 0)),
        new Pose2d(4,0, new Rotation2d(0)),
        config);


        Trajectory fourMeterTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(4,0, new Rotation2d(0)),
            config);

        var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0,
        AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new
        SwerveControllerCommand(
        fourMeterTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

        // // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(fourMeterTrajectory.getInitialPose());

        // This will load the file "Example Path.path" and generate it with a max
        // velocity of 4 m/s and a max acceleration of 3 m/s^2
        // // Run path following command, then stop at the end
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, true, true));
        // }
        // autoChooser.addOption("Mobility", new InstantCommand());
        // SmartDashboard.putData("autos/Auto Chooser", autoChooser);
    }
}
