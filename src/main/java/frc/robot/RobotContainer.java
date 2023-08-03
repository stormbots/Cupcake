// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.List;

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
                -MathUtil.applyDeadband(Math.pow(m_driverController.getLeftY(), 2)*Math.signum(m_driverController.getLeftY()), OIConstants.kDriveDeadband), //squaring inputs to make robot more controllable
                -MathUtil.applyDeadband(Math.pow(m_driverController.getLeftX(), 2)*Math.signum(m_driverController.getLeftX()), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(), OIConstants.kDriveDeadband),
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
            ()-> m_robotDrive.zeroHeading(),
            m_robotDrive));

    new JoystickButton(m_driverController, 7)
        .whileTrue(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        // RobotRelative below has not been tested
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(Math.pow(m_driverController.getLeftY(), 2)*Math.signum(m_driverController.getLeftY()), OIConstants.kDriveDeadband), //squaring inputs to make robot more controllable
                -MathUtil.applyDeadband(Math.pow(m_driverController.getLeftX(), 2)*Math.signum(m_driverController.getLeftX()), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(), OIConstants.kDriveDeadband),
                false, true),
            m_robotDrive));
  }


  // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2


// This trajectory can then be passed to a path follower such as a PPSwerveControllerCommand
// Or the path can be sampled at a given point in time for custom path following

public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
   return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              m_robotDrive.resetOdometry(traj.getInitialHolonomicPose());
          }
        }),
        new PPSwerveControllerCommand(
            traj, 
            m_robotDrive::getPose, // Pose supplier
            DriveConstants.kDriveKinematics, // SwerveDriveKinematics
            new PIDController(AutoConstants.kPXController, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(AutoConstants.kPYController, 0, 0), // Y controller (usually the same values as X controller)
            new PIDController(AutoConstants.kPThetaController, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            m_robotDrive::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            m_robotDrive // Requires this drive subsystem
        )
    );
}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Run path following command, then stop at the end.
    return FollowTrajectoryCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, true, true));
  }
}
