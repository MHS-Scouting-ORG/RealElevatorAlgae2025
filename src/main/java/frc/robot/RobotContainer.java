package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorCmds.ManualElevatorCmd;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.commands.AlgaePivotCmds.ManualPivotCmd;
import frc.robot.commands.AlgaePivotCmds.StoragePositionCmd;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlgaeIntakeCmds.IntakeCmd;
import frc.robot.commands.AlgaeIntakeCmds.OuttakeCmd;
import frc.robot.commands.IntegratedCmds.TuckCmd;
import frc.robot.commands.AlgaePivotCmds.ProcessorPositionCmd;
import frc.robot.commands.AlgaePivotCmds.StoragePositionCmd;

public class RobotContainer {

  public final XboxController xbox = new XboxController(OperatorConstants.kDriverControllerPort);

  public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public final AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();

  public Command disableAlgaeIntakePID = new InstantCommand(() -> algaeIntakeSubsystem.disablePID(), algaeIntakeSubsystem);
  
  public RobotContainer() {
 
    // elevatorSubsystem.setDefaultCommand(new ManualElevatorCmd(elevatorSubsystem, () -> xbox.getLeftY()));

    //joystick control for moving the algae pivot manually
    algaeIntakeSubsystem.setDefaultCommand(new ManualPivotCmd(algaeIntakeSubsystem, () -> xbox.getRightY()));
    configureBindings();
  }

  private void configureBindings() {

    //button control to move the algae pivot to the storage position
    new JoystickButton(xbox, XboxController.Button.kB.value).onTrue(new ProcessorPositionCmd(algaeIntakeSubsystem));
    // new JoystickButton(xbox, XboxController.Button.kA.value).onTrue(new StoragePositionCmd(algaeIntakeSubsystem));


    // new JoystickButton(xbox, XboxController.Button.kY.value).onTrue(new InstantCommand(() -> algaeIntakeSubsystem.runIntakeMotor(0.1), algaeIntakeSubsystem));
    new JoystickButton(xbox, XboxController.Button.kY.value).onTrue(new IntakeCmd(algaeIntakeSubsystem));
    new JoystickButton(xbox, XboxController.Button.kX.value).whileTrue(new OuttakeCmd(algaeIntakeSubsystem));
    // new JoystickButton(xbox, XboxController.Button.kLeftBumper.value).onTrue(new TuckCmd());
  }

  public Command getAutonomousCommand() {
    return Autos.exampleAuto();
  }

  public Command disableAlgaeIntakePID(){
    return disableAlgaeIntakePID;
  }
}
