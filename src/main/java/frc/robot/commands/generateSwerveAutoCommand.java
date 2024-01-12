// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Snippets;
import frc.robot.subsystems.Chassis;

public class generateSwerveAutoCommand extends CommandBase {
  /** Creates a new generateSwerveAutoCommand. */

  Pose2d targetPose;
  Snippets snippets; 
  Chassis chassis;
  SwerveControllerCommand swerveControllerCommand;


  public generateSwerveAutoCommand(Pose2d pose, Snippets snippets, Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.snippets = snippets;
    this.chassis = chassis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveControllerCommand = snippets.generateSwerveControllerCommand(snippets.getTrajectoryTo(targetPose));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
