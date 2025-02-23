// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.Console;
import java.io.File;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
 
  double maximumSpeed = Units.feetToMeters(4.5);
  File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive  swerveDrive;


  public SwerveSubsystem() {

    try{
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(4.5);
    }catch(Exception e){
      throw new RuntimeException(e);
    }

    

  }

 
  public Command exampleMethodCommand() {
    
    return runOnce(
        () -> {

        });
  }

  
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

      System.out.println("Module positions: "+
        Math.round(swerveDrive.getModules()[0].getAbsolutePosition()) + " ___" +
        Math.round(swerveDrive.getModules()[1].getAbsolutePosition()) + " ___" +
        Math.round(swerveDrive.getModules()[2].getAbsolutePosition()) + " ___" +
        Math.round(swerveDrive.getModules()[3].getAbsolutePosition())

      );


  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void driveFieldOriented(ChassisSpeeds velocity){
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
    return run(()-> {
      swerveDrive.driveFieldOriented(velocity.get());
    }
    );
  }







}
