// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AlgaePivotCmds.StoragePosition2Cmd;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public final class Autos {


  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(AlgaeIntakeSubsystem newAlgaeIntakeSubsystem) {
    return new StoragePosition2Cmd(newAlgaeIntakeSubsystem);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
