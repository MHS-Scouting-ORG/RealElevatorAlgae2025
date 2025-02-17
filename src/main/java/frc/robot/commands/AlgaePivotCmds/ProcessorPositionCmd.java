// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaePivotCmds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ProcessorPositionCmd extends Command {
  AlgaeIntakeSubsystem algaeIntakeSub;
  Timer timer;

  public ProcessorPositionCmd(AlgaeIntakeSubsystem newAlgaeIntakeSub) {
    algaeIntakeSub = newAlgaeIntakeSub;
    timer = new Timer();
    addRequirements(algaeIntakeSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    algaeIntakeSub.setSetpoint(-1100); //-1350
    algaeIntakeSub.enablePID();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (algaeIntakeSub.isDone()){
    //   timer.start();
    //   if (timer.get() >= 1){
    //     double newOutput = algaeIntakeSub.getOutput();
    //     algaeIntakeSub.disablePID();
    //     algaeIntakeSub.setOutput(newOutput);
    //     timer.stop();
    //     timer.reset();
    //   }
    // }
    // else{
    //   algaeIntakeSub.enablePID();
    //   timer.reset();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
