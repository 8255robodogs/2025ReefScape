package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SparkMaxMotor;
import frc.robot.subsystems.VictorSPXMotorSubsystem;

public class CmdSpinMotorPositive extends Command{
    private SparkMaxMotor motor;
    private double secondsToSpin;
    private long startTime;
    private float secondsElapsed = 0.0f;
    

    public CmdSpinMotorPositive( double seconds, SparkMaxMotor motorSubsystem){
        secondsToSpin = seconds;
        this.motor = motorSubsystem;
        
    }

    @Override
    public void initialize(){
        startTime = System.nanoTime();
        
    }

    @Override 
    public void execute(){
        motor.setSpeed(.3);
        
        //time tracking
        float currentTime = System.nanoTime();
        float nanoSecondsElapsed = currentTime - startTime;
        secondsElapsed = nanoSecondsElapsed / 1000000000; //divide by 1 billion
        System.out.println("DEBUG. nanosecondsElapsed: "+ nanoSecondsElapsed+ " ElapsedTime: "+ secondsElapsed);    
    }

    @Override
    public boolean isFinished(){
        System.out.println("secondsElapsed: "+secondsElapsed+" goal: "+secondsToSpin);
        //we have to return a true value at the end of isFinished to move onto the "end" function
        if(secondsElapsed >= secondsToSpin){
            return true;
        }else{
            return false;
        }
        
    }

    @Override
    public void end(boolean interrupted){
        motor.setSpeed(0);
    }

 

}
