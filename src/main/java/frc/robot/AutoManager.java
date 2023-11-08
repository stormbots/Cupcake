// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    autoChooser.addOption("example trajectory", snippets.generateTrajectory(snippets.exampleTrajectory) .andThen(() -> chassis.drive(0, 0, 0, true, true)));
    SmartDashboard.putData("autos/Auto Chooser", autoChooser);
  }

  public Command getAutonomusCommand(){
    return autoChooser.getSelected();
  }
}
