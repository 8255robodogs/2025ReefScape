package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberDownCmd;
import frc.robot.commands.ClimberUpCmd;
import frc.robot.subsystems.ReefscapeAlgaeSubsystem;
import frc.robot.subsystems.ReefscapeClimbSubsystem;
import frc.robot.subsystems.ReefscapeElevatorSubsystem;
import frc.robot.subsystems.ReefscapeHeadSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class RobotContainer {
  
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final ReefscapeElevatorSubsystem elevator = new ReefscapeElevatorSubsystem();
  private final ReefscapeHeadSubsystem head = new ReefscapeHeadSubsystem();
  private final ReefscapeAlgaeSubsystem algaeSystem = new ReefscapeAlgaeSubsystem();
  private final ReefscapeClimbSubsystem climber = new ReefscapeClimbSubsystem();

  //declare the controller
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
  SendableChooser<Command> autoSelector = new SendableChooser<Command>();
  
  SendableChooser<Pose2d> startingPositionSelector = new SendableChooser<>();
    private Pose2d blueLeft = new Pose2d(7.386,7.275, new Rotation2d(0));
    private Pose2d blueMiddle = new Pose2d(7.120,5.66, new Rotation2d(0));
    private Pose2d blueRight = new Pose2d(7.788,5.069, new Rotation2d(0));
    
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
    () -> m_driverController.getLeftY() *1,
    () -> m_driverController.getLeftX() *1)
    .withControllerRotationAxis(m_driverController::getRightX)
    .deadband(OperatorConstants.kDriverStickDeadband)
    .scaleTranslation(1)
    .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
    .withControllerHeadingAxis(m_driverController::getRightX, 
    m_driverController::getRightY)
    .headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);


  
  public RobotContainer() {
    //Explains the robot to pathplanner so it can be used to drive paths.
    drivebase.setupPathPlanner();
    
    //registers controls
    configureBindings();
    configureAutos();
  }
  
  private void configureBindings() {
    
    //DRIVER CONTROLLER (CONTROLLER ZERO)

    //driving controls

    //this makes the drivebase drive. Very important.
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    //hold X to lock the wheels and resist being pushed
    m_driverController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //press start to zero your heading
    m_driverController.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
    



    //algae systems - collection and scoring
    
    //Pressing A will lower the algae harvester and turn on the wheels to suck algae in
    m_driverController.a().onTrue(algaeSystem.setAlgaeCollectorPistonExtended(true)
    .alongWith(algaeSystem.setAlgaeCollectorMotorSpeed(1)));
    
    //Releasing A will raise the algae harvester and reduce the wheels speed to just hold the algae
    m_driverController.a().onFalse(algaeSystem.setAlgaeCollectorPistonExtended(false)
    .alongWith(algaeSystem.setAlgaeCollectorMotorSpeed(0.2)));
    
    //Pressing B will set the algae harvester wheels in full reverse to spit the algae out
    m_driverController.b().onTrue(algaeSystem.setAlgaeCollectorMotorSpeed(-1));
    
    //Releasing B will turn the algae harvester wheels off
    m_driverController.b().onFalse(algaeSystem.setAlgaeCollectorMotorSpeed(0));

    //algae systems - remover tool
    
    //toggle the remover
    m_driverController.y().onTrue(algaeSystem.toggleAlgaeRemover());









    //OPERATOR CONTROLLER (CONTROLLER ONE)

    //elevator
    m_operatorController.a().onTrue(elevator.setSetpointCommand(1));
    m_operatorController.x().onTrue(elevator.setSetpointCommand(2));
    m_operatorController.b().onTrue(elevator.setSetpointCommand(3));
    m_operatorController.y().onTrue(elevator.setSetpointCommand(4)); 

    //head
    head.setDefaultCommand(head.setHeadSpeed(0));
    m_operatorController.rightBumper().onTrue(head.setHeadSpeed(0.4));
    m_operatorController.leftBumper().onTrue(head.setHeadSpeed(-0.4));
    m_operatorController.rightTrigger(0.1).whileTrue(head.setHeadSpeed(m_operatorController.getRightTriggerAxis()));
    m_operatorController.leftTrigger(0.1).whileTrue(head.setHeadSpeed(m_operatorController.getLeftTriggerAxis()*-1));

    //lifter
    m_operatorController.start().onTrue(new ClimberUpCmd(climber));
    m_operatorController.back().onTrue(new ClimberDownCmd(climber));


  } 

  private void configureAutos(){
    /*
    SmartDashboard.putData(startingPositionSelector);
    startingPositionSelector.setDefaultOption("blueMiddle", blueMiddle);
    startingPositionSelector.addOption("blueLeft",blueLeft);
    startingPositionSelector.addOption("blueMiddle", blueMiddle);
    startingPositionSelector.addOption("blueRight", blueRight);
    */
    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
    return AutoBuilder.buildAuto("Test Auto")
    
    ;

    /*
    Pose2d startPosition = new Pose2d(
      new Translation2d(0,0),
      new Rotation2d(10)
    );

    Pose2d destination1 = new Pose2d(
      new Translation2d(0.3,0),
      new Rotation2d(10)
    );
    
    PathConstraints constraints = new PathConstraints(1,0.5,0.1,0.1,0.1);

    return AutoBuilder.resetOdom(startPosition)
    .andThen(AutoBuilder.pathfindToPose(destination1,constraints))
    .andThen(new CmdSetElevatorHeight(giraffeNeckMotor, 3, 0.3))
    .andThen(new CmdSpinMotorPositive(1, coralFeeder))
    ;
    
    
    //return autoSelector.getSelected();
    */
  }
}
