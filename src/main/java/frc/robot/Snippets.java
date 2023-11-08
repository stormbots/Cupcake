// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Chassis;

public class Snippets {
  CommandJoystick operator;
  CommandJoystick driver;
  Chassis chassis;

  /** Creates a new Snippets. */
  public Snippets(CommandJoystick operator, CommandJoystick driver, Chassis chassis) {
    this.operator = operator; 
    this.driver = driver; 
    this.chassis = chassis;
  }

  public static final Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(1,0, new Rotation2d(0)),
      Constants.getTrajectoryConfig());
  
  SwerveControllerCommand generateTrajectory(Trajectory trajectory){
    
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //   new Pose2d(0, 0, new Rotation2d(0)),
    //   List.of(),
    //   new Pose2d(1,0, new Rotation2d(0)),
    //   config);

    SwerveControllerCommand swerveControllerCommand = new
    SwerveControllerCommand(
      trajectory,
      chassis::getPose, // Functional interface to feed supplier
      DriveConstants.kDriveKinematics,
      // Position controllers
      new PIDController(AutoConstants.kPXController, 0, 0),
      new PIDController(AutoConstants.kPYController, 0, 0),
      Constants.getPIDController(),
      chassis::setModuleStates,
      chassis
    );

    return swerveControllerCommand;

  }


  public Command getCommandExample(){
    return new InstantCommand();
  }


  // autoChooser.setDefaultOption("Do Nothing", new InstantCommand());


}
