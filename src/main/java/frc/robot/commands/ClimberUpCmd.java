package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ReefscapeClimbSubsystem;

public class ClimberUpCmd extends Command {

    private final ReefscapeClimbSubsystem climber;
    
    private double secondsBeforeLockingPiston = 0.0;
    private double secondsdBeforeClimbingPiston = 0.2;
    private long startTime;
    private float secondsElapsed = 0.0f;

    public ClimberUpCmd(ReefscapeClimbSubsystem climber) {
        this.climber = climber;        
    }

    @Override
    public void initialize() {
        startTime = System.nanoTime();
    }

    @Override
    public void execute() {

        //time tracking
        float currentTime = System.nanoTime();
        float nanoSecondsElapsed = currentTime - startTime;
        secondsElapsed = nanoSecondsElapsed / 1000000000; //divide by 1 billion to turn nanoseconds into seconds

        if(secondsElapsed >= secondsBeforeLockingPiston){
            climber.lockingPiston.set(true);
        }

        if(secondsElapsed >= secondsdBeforeClimbingPiston){
            climber.climbPiston.set(Value.kForward);
        }

    }

    

    @Override
    public boolean isFinished() {

        return climber.climbPiston.get() == Value.kForward;

    }

    @Override
    public void end(boolean interrupted) {

    }
    
}
