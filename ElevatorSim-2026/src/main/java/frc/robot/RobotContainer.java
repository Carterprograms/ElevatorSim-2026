// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.ElevatorCmd;
import frc.robot.Subsystmes.ElevatorSys;

public class RobotContainer {

  // Instantiation of subsystem
  private final ElevatorSys elevatorSys = new ElevatorSys();

  // Instantiation of controller
  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    // Apply configured bindings
    configureBindings();
  }

  private void configureBindings() {
    // Configure binding from controller to execute command
    controller.a().whileTrue(new ElevatorCmd(elevatorSys));
  }

  public Command getAutonomousCommand() {
    // No autonomous to return
    return Commands.print("No autonomous command configured");
  }
}
