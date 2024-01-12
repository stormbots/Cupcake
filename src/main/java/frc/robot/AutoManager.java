// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Chassis;

public class AutoManager {
  /** Creates a new AutosSubsystem. */
  Snippets snippets;
  Chassis chassis;

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  public AutoManager(Snippets snippets, Chassis chassis) {
    this.snippets = snippets;
    this.chassis = chassis;
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("example case", snippets.getCommandExample());
    autoChooser.addOption("example trajectory", snippets.generateSwerveControllerCommand(Snippets.exampleTrajectory) .andThen(() -> chassis.drive(0, 0, 0, true, true)));
    autoChooser.addOption("example trajectory forward left", snippets.generateSwerveControllerCommand(Snippets.exampleTrajectory)
      .andThen(
        snippets.generateSwerveControllerCommand(snippets.getTrajectoryTo(new Pose2d(1,1,new Rotation2d(0))))
      )
      );
    // autoChooser.addOption("example pathweaver", snippets.generateTrajectory(snippets.testtrajectory).andThen(() -> snippets.generateTrajectory(Snippets.exampleTrajectory)).andThen(() -> chassis.drive(0, 0, 0, true, true)));
    //NO WORK, thinking is just completely wrong
    // autoChooser.addOption("example pathweaver with sequencing", 
    // snippets.generateTrajectory(snippets.testtrajectory)
    // .andThen(() -> chassis.drive(0, 0, 0, true, true)).withTimeout(1)
    // .andThen(snippets.generateTrajectory(TrajectoryGenerator.generateTrajectory(
    //   chassis.getPose(),
    //   List.of(),
    //   new Pose2d(1,0, new Rotation2d(0)),
    //   Constants.getTrajectoryConfig()))));

    SmartDashboard.putData("autos/Auto Chooser", autoChooser);
  }

  public Command getAutonomusCommand(){
    return autoChooser.getSelected();
  }
}
