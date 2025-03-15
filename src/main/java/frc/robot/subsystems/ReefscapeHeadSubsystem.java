package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReefscapeHeadSubsystem extends SubsystemBase{

    //Motor settings
    private VictorSPX motor;
    private int motorControllerCanID = 14;
    private boolean invertedMotor = false;
    
    //Constructor
    public ReefscapeHeadSubsystem(){
        motor = new VictorSPX(motorControllerCanID);
    }

    public void setMotorSpeed(double speed){
        double newSpeed = speed;
        if(invertedMotor == true){
            newSpeed = newSpeed *-1;
        }
        motor.set(VictorSPXControlMode.PercentOutput, newSpeed);
    }


    @Override
    public void periodic(){
        
    }

    public Command setHeadSpeed(double speed){
        return Commands.run(() -> setMotorSpeed(speed),this);
    }

    

}



