package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ElevatorCmds.ManualElevatorCmd;
import frc.robot.commands.ElevatorCmds.TestPIDCmd;
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
import frc.robot.subsystems.AlgaePivotSubsystem;

public class RobotContainer {

  public final XboxController xbox = new XboxController(OperatorConstants.kDriverControllerPort);

  public final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  public final AlgaeIntakeSubsystem algaeIntakeSubsystem = new AlgaeIntakeSubsystem();
  public final AlgaePivotSubsystem algaePivotSubsystem = new AlgaePivotSubsystem();

  public RobotContainer() {
 
    elevatorSubsystem.setDefaultCommand(new ManualElevatorCmd(elevatorSubsystem, () -> xbox.getLeftY()));

    //joystick control for moving the algae pivot manually
    algaePivotSubsystem.setDefaultCommand(new ManualPivotCmd(algaePivotSubsystem, () -> xbox.getRightY()));

    algaeIntakeSubsystem.setDefaultCommand(new InstantCommand(() -> algaeIntakeSubsystem.runIntakeMotor(0.3), algaeIntakeSubsystem));
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(xbox, XboxController.Button.kA.value).onTrue(new TestPIDCmd(elevatorSubsystem, 100));
    // new JoystickButton(xbox, XboxController.Button.kB.value).onTrue(new TestPIDCmd(elevatorSubsystem, 0));

    //button control to move the algae pivot to the storage position
    new JoystickButton(xbox, XboxController.Button.kB.value).onTrue(new StoragePositionCmd(algaePivotSubsystem));

    new JoystickButton(xbox, XboxController.Button.kY.value).onTrue(new IntakeCmd(algaeIntakeSubsystem));
    new JoystickButton(xbox, XboxController.Button.kX.value).whileTrue(new OuttakeCmd(algaeIntakeSubsystem));
  }

  public Command getAutonomousCommand() {
    return Autos.exampleAuto();
  }
}
