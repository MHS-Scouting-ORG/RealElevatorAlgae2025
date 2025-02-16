// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntegratedCmds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.ElevatorCmds.ElevProcessorPos;
import frc.robot.commands.AlgaePivotCmds.ProcessorPositionCmd;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ProcessorScoring extends SequentialCommandGroup {
  AlgaeIntakeSubsystem algaeIntakeSub;
  ElevatorSubsystem elevatorSub;
  //test commenty\
  public ProcessorScoring(AlgaeIntakeSubsystem newAlgaeIntakeSub, ElevatorSubsystem newElevatorSub) {
    algaeIntakeSub = newAlgaeIntakeSub;
    elevatorSub = newElevatorSub;

    addCommands(new ElevProcessorPos(elevatorSub), new ProcessorPositionCmd(algaeIntakeSub));
  }
}
