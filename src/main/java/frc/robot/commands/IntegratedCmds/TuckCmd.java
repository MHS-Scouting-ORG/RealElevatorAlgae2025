package frc.robot.commands.IntegratedCmds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlgaePivotCmds.NoAlgaeTuckCmd;
import frc.robot.commands.ElevatorCmds.BottomElevPos;
import frc.robot.commands.ElevatorCmds.ElevProcessorPos;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;   

public class TuckCmd extends SequentialCommandGroup {
  AlgaeIntakeSubsystem algaeIntakeSub;
  ElevatorSubsystem elevatorSub;
  // test comment
  public TuckCmd(AlgaeIntakeSubsystem newAlgaeIntakeSub, ElevatorSubsystem newElevatorSub) {
    algaeIntakeSub = newAlgaeIntakeSub;
    elevatorSub = newElevatorSub;
    addCommands(new ElevProcessorPos(elevatorSub), new NoAlgaeTuckCmd(algaeIntakeSub), new BottomElevPos(elevatorSub));
    addRequirements(algaeIntakeSub, elevatorSub);
  }
}
