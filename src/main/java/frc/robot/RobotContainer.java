package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ReefscapeElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class RobotContainer {
  
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final ReefscapeElevatorSubsystem elevator = new ReefscapeElevatorSubsystem();

  //declare the controller
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
    () -> m_driverController.getLeftY() *1,
    () -> m_driverController.getLeftX() *1)
    .withControllerRotationAxis(m_driverController::getRightX)
    .deadband(OperatorConstants.kDriverStickDeadband)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                            .withControllerHeadingAxis(m_driverController::getRightX, 
                                                                       m_driverController::getRightY)
                                                                       .headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);


  
  public RobotContainer() {
    //registers controls
    configureBindings();

    //registers the names of commands so they can be found in Path Planner
    //NamedCommands.registerCommand("goToTopPoleHeight", Commands.runOnce(elevator::goToTopPoleHeight));
    //NamedCommands.registerCommand("goToMiddlePoleHeight", Commands.runOnce(elevator::goToMiddlePoleHeight));
    //NamedCommands.registerCommand("goToBottomPoleHeight", Commands.runOnce(elevator::goToBottomPoleHeight));
    //NamedCommands.registerCommand("goToPickupHeight", Commands.runOnce(elevator::goToPickupHeight));
  }
  
  private void configureBindings() {
    
    //driver controller
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    m_driverController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    m_driverController.y().onTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
    m_driverController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    

    //operator controller
    //Neck auto mode
    //m_operatorController.a().onTrue(Commands.runOnce(elevator::goToPickupHeight));
    //m_operatorController.x().onTrue(Commands.runOnce(elevator::goToBottomPoleHeight));
    //m_operatorController.b().onTrue(Commands.runOnce(elevator::goToMiddlePoleHeight));
    //m_operatorController.y().onTrue(Commands.runOnce(elevator::goToTopPoleHeight));
    
    

  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
