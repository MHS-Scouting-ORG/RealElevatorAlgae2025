// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntegratedCmds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaeIntakeCmds.IntakeCmd;
import frc.robot.commands.AlgaePivotCmds.ProcessorPositionCmd;
import frc.robot.commands.ElevatorCmds.L2ElevPos;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L2Dealgify extends SequentialCommandGroup {
  
  private ElevatorSubsystem elevatorSubsystem;
  private AlgaeIntakeSubsystem algaeIntakeSubsystem;

  /** Creates a new L2Dealgify. */
  public L2Dealgify(ElevatorSubsystem newElevatorSubsystem, AlgaeIntakeSubsystem newAlgaeIntakeSubsystem) {

    ElevatorSubsystem elevatorSubsystem = newElevatorSubsystem;
    AlgaeIntakeSubsystem algaeIntakeSubsystem = newAlgaeIntakeSubsystem;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new L2ElevPos(elevatorSubsystem),
      new ProcessorPositionCmd(algaeIntakeSubsystem),
      new IntakeCmd(algaeIntakeSubsystem),
      new TuckWithAlgae()
    );
  }
}
