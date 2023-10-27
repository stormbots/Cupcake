// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoManager {
  /** Creates a new AutosSubsystem. */
  Snippets snippets;

  SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  public AutoManager(Snippets snippets) {
    this.snippets = snippets;
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("example case", snippets.getCommandExample());
    SmartDashboard.putData("autos/Auto Chooser", autoChooser);
  }

  public Command getAutonomusCommand(){
    return autoChooser.getSelected();
  }
}
