package frc.robot.commands;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SparkMaxMotor;
import frc.robot.subsystems.VictorSPXMotorSubsystem;

public class CmdSetElevatorHeight extends Command{
    private SparkMaxMotor motor;
    private int level;
    private double speed;
    private double[] levels = {0,25.5,79,180};

    private double errorTolerance = 1;

    public CmdSetElevatorHeight(SparkMaxMotor motor, int level, double speed){
        this.motor = motor;
        this.level = level;
        this.speed = speed;
    }

    @Override
    public void initialize(){

    }

    @Override 
    public void execute(){
    
        if(motor.getEncoderDegrees() < levels[level]){
            motor.setSpeed(speed);
        }else{
            motor.setSpeed(-speed);
        }
    
        
    }

    @Override
    public boolean isFinished(){
        
        return Math.abs(motor.getEncoderDegrees() - levels[level]) < errorTolerance;
        
    }

    @Override
    public void end(boolean interrupted){
        motor.setSpeed(0);
    }

}




