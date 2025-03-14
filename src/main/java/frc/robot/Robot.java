// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.features2d.AgastFeatureDetector;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.programs.AutoReefscapeTest;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.SparkMaxMotor;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VictorSPXMotorSubsystem;
import swervelib.SwerveDrive;

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
  private PneumaticSubsystem cagePneumatic;
  private PneumaticSubsystem algaePneumatic;
  private Solenoid algaeRemoverPneumatic;
  private PneumaticsControlModule pcm;

  private VictorSPXMotorSubsystem coralMotor;
  private VictorSPXMotorSubsystem harvestorMotor;
  private SparkMaxMotor giraffeNeckMotor;
  

  private final double heightForTopPole = 180;
    private final double heightForMiddlePole = 79;
    private final double heightForBottomPole = 25.5;
    private final double heightForPickup = 0;
  



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
    harvestorMotor = new VictorSPXMotorSubsystem(5, "harvestorMotor", true);
    
    cagePneumatic = new PneumaticSubsystem(21, 2, 3, true);
    algaePneumatic = new PneumaticSubsystem(21,4, 5, false);
    algaeRemoverPneumatic = new Solenoid(21,PneumaticsModuleType.CTREPCM,1);
    algaeRemoverPneumatic.set(false);


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

    //set up the autonomous command
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


    //ELEVATOR / NECK LIFTING
    if(xbox1.getAButton() & !xbox1.getYButton() & (giraffeNeckMotor.getEncoderDegrees() > 0 || xbox1.getLeftTriggerAxis() > 0.1)){
      //neck down
      giraffeNeckMotor.setSpeed(-1);
    }else if(!xbox1.getAButton() & xbox1.getYButton() & (giraffeNeckMotor.getEncoderDegrees() < 180 || xbox1.getLeftTriggerAxis() > 0.1)){
      //neck up`
      giraffeNeckMotor.setSpeed(1); //was 32
    }else if(xbox1.getXButton()){
      if(giraffeNeckMotor.getEncoderDegrees()< heightForBottomPole){
        giraffeNeckMotor.setSpeed((1));
      }else{
        giraffeNeckMotor.setSpeed(-1);
      }
    }else if(xbox1.getBButton()){
      if(giraffeNeckMotor.getEncoderDegrees()< heightForMiddlePole){
        giraffeNeckMotor.setSpeed((1));
      }else{
        giraffeNeckMotor.setSpeed(-1);
      }
    }
    else{
      giraffeNeckMotor.setSpeed(0);
    }

    if(xbox1.getRightStickButtonPressed()){
      giraffeNeckMotor.resetEncoder();
    }
    
    //needed to reset neck if the starts at a non nutral height
    if(xbox1.getBackButton() ){
      giraffeNeckMotor.setSpeed(-0.3);
    }
    

    //coral motor
    if(xbox1.getRightBumperButton() && !xbox1.getLeftBumperButton()){
      coralMotor.SetSpeed(0.3);
    }else if(!xbox1.getRightBumperButton() && xbox1.getLeftBumperButton()){
      coralMotor.SetSpeed(-0.3);
    }else if(xbox0.getRightBumperButton()){
      coralMotor.SetSpeed(.4);
    }else if(xbox1.getRightTriggerAxis() > 0.5){
      coralMotor.SetSpeed(1);
    }
    else{
      coralMotor.SetSpeed(0);
    }

    //harvestor motor
    if(xbox0.getXButton() && !xbox0.getLeftBumperButton()){
      harvestorMotor.SetSpeed(-1);
    }else if(!xbox0.getXButton() && xbox0.getLeftBumperButton())
      harvestorMotor.SetSpeed(.5);
    
    else{
      harvestorMotor.SetSpeed(0);
    }



    //harvestor piston
    if(xbox0.getAButtonPressed()){
      algaePneumatic.TogglePneumatic();
    }
    


    //cage liftpiston
    if(xbox1.getStartButtonPressed()){
      cagePneumatic.TogglePneumatic();
    }

    //algae knocker piston
    if(xbox0.getYButtonPressed()){
      algaeRemoverPneumatic.toggle();
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
