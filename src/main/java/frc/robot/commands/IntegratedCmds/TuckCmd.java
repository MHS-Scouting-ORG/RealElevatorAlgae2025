package frc.robot.commands.IntegratedCmds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaePivotCmds.StoragePositionCmd;
import frc.robot.commands.ElevatorCmds.BottomElevPos;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class TuckCmd extends SequentialCommandGroup {
  public TuckCmd() {
    addCommands(new StoragePositionCmd(new AlgaeIntakeSubsystem()), new BottomElevPos(new ElevatorSubsystem()));
  }
}
