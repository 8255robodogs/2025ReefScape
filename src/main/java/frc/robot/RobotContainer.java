package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.CmdSpinMotorPositive;
import frc.robot.commands.auto.programs.AutoReefscapeTest;
import frc.robot.commands.auto.programs.Forward;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ReefscapeElevatorSubsystem;
import frc.robot.subsystems.SparkMaxMotor;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class RobotContainer {
  
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final SparkMaxMotor coralFeeder = new SparkMaxMotor(14, false);
  //private final ReefscapeElevatorSubsystem elevator = new ReefscapeElevatorSubsystem();

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
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                            .withControllerHeadingAxis(m_driverController::getRightX, 
                                                                       m_driverController::getRightY)
                                                                       .headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);


  
  public RobotContainer() {
    drivebase.setupPathPlanner();
    //registers controls
    configureBindings();
    configureAutos();
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
    m_driverController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    

    

  }

  private void configureAutos(){
    SmartDashboard.putData("auto selector", autoSelector);
    autoSelector.setDefaultOption("Do Nothing", null);
    autoSelector.addOption("AutoReefscapeTest", new AutoReefscapeTest(drivebase) );
    autoSelector.addOption("Forward", new Forward(drivebase) );
    

    SmartDashboard.putData(startingPositionSelector);
    startingPositionSelector.setDefaultOption("blueMiddle", blueMiddle);
    startingPositionSelector.addOption("blueLeft",blueLeft);
    startingPositionSelector.addOption("blueMiddle", blueMiddle);
    startingPositionSelector.addOption("blueRight", blueRight);
    
    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
    return AutoBuilder.buildAuto("Test Auto");
/*
    Pose2d startPosition = new Pose2d(
      new Translation2d(0,0),
      new Rotation2d(10)
    );

    Pose2d destination1 = new Pose2d(
      new Translation2d(0.3,0),
      new Rotation2d(10)
    );
    //3.03 1.35

    PathConstraints constraints = new PathConstraints(1,0.5,0.1,0.1,0.1);

    /*
    return AutoBuilder.resetOdom(startPosition)
    .andThen(AutoBuilder.pathfindToPose(destination1,constraints))
    .andThen(new CmdSpinMotorPositive(1, coralFeeder))
    ;
    */
    
    //return autoSelector.getSelected();
    
  }
}
