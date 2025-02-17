// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntegratedCmds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaePivotCmds.AlgaeTuckCmd;
import frc.robot.commands.ElevatorCmds.ElevProcessorPos;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TuckWithAlgae extends SequentialCommandGroup {
  AlgaeIntakeSubsystem algaeIntakeSub;
  ElevatorSubsystem elevatorSub;
  // testing comment
  public TuckWithAlgae(AlgaeIntakeSubsystem newAlgaeIntakeSub, ElevatorSubsystem newElevatorSub) {
    algaeIntakeSub = newAlgaeIntakeSub;
    elevatorSub = newElevatorSub;
    addCommands(new ElevProcessorPos(elevatorSub), new AlgaeTuckCmd(algaeIntakeSub));
    addRequirements(algaeIntakeSub, elevatorSub);
  }
}
