// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.features2d.AgastFeatureDetector;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.SparkMaxMotor;
import frc.robot.subsystems.VictorSPXMotorSubsystem;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  XboxController xbox0 = new XboxController(0);
  XboxController xbox1 = new XboxController(1);
  private SparkMaxMotor giraffeNeckMotor;
  private PneumaticSubsystem cagePneumatic;
  private PneumaticSubsystem algaePneumatic;
  private PneumaticsControlModule pcm;

  private VictorSPXMotorSubsystem coralMotor;
  

  SendableChooser<Command> autoSelector = new SendableChooser<Command>();


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    giraffeNeckMotor = new SparkMaxMotor(9,true); 
    pcm = new PneumaticsControlModule(21);
    pcm.clearAllStickyFaults();
    coralMotor = new VictorSPXMotorSubsystem(14, "coralMotor", false);
    
    cagePneumatic = new PneumaticSubsystem(21, 2, 3, true);
    algaePneumatic = new PneumaticSubsystem(21,4, 5, false);

    SmartDashboard.putData("auto selector", autoSelector);
        autoSelector.setDefaultOption("Do Nothing", null);
        //autoSelector.addOption("Auto Shoot And Stay Still", new AutoShootAndStayStill(swerveSys, launcherMotorA,launcherMotorB,lifterGate,groundPickupMotor,LauncherFeeder));
        

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    


  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }


  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {


    //giraffe neck
    //top positoion 59.9
    //middle rack 25





    if(xbox1.getAButton() & !xbox1.getYButton() & (giraffeNeckMotor.getEncoderDegrees() > 0 || xbox1.getLeftTriggerAxis() > 0.1)){
      //neck down
      giraffeNeckMotor.setSpeed(-0.35);
    }else if(!xbox1.getAButton() & xbox1.getYButton() & (giraffeNeckMotor.getEncoderDegrees() < 59.9 || xbox1.getLeftTriggerAxis() > 0.1)){
      //neck up`
      giraffeNeckMotor.setSpeed(0.32);
    }else{
      giraffeNeckMotor.setSpeed(0);
    }
    
    if(xbox1.getLeftStickButtonPressed()){
      giraffeNeckMotor.resetEncoder();
    }

    System.out.println("Encoder: "+giraffeNeckMotor.getEncoderDegrees());
    

    
    //coral motor
    if(xbox1.getRightBumperButton() && !xbox1.getLeftBumperButton()){
      coralMotor.SetSpeed(0.3);
    }else if(!xbox1.getRightBumperButton() && xbox1.getLeftBumperButton()){
      coralMotor.SetSpeed(-0.3);
    }else{
      coralMotor.SetSpeed(0);
    }



    //algae piston
    if(xbox1.getBButtonPressed()){
      //algaePneumatic.TogglePneumatic();
    }
    

    //algae motor

    //cage piston
    if(xbox1.getXButtonPressed()){
      cagePneumatic.TogglePneumatic();
    }


  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
