// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.Chassis;

public class OperatorInterface {

  CommandJoystick operator;
  CommandJoystick driver;
  Snippets snippets;
  Chassis chassis;

  /** Creates a new Operator. */
  public OperatorInterface(CommandJoystick operator, CommandJoystick driver, Snippets snippets){
    this.operator = operator;
    this.driver = driver;
    this.snippets = snippets;
  }
}
