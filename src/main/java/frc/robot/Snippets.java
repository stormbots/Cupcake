// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
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

  public Command getCommandExample(){
    return new InstantCommand();
  }


  // autoChooser.setDefaultOption("Do Nothing", new InstantCommand());


}
