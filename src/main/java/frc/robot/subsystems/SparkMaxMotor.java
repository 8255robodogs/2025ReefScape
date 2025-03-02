package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxMotor extends SubsystemBase{
    
    SparkMax motor;
    boolean inverted = false;

    public SparkMaxMotor(int canID, boolean inverted){
        motor = new SparkMax(canID,MotorType.kBrushless);
        this.inverted = inverted;
    }

    public void setSpeed(double speed){
        if (inverted) speed = speed *-1;
        motor.set(speed);
    }

    public double getSpeed(){
        double speed= motor.get();
        if (inverted) speed = speed *-1;
        return speed;
    }




    public double getEncoderDegrees(){
        return motor.getEncoder().getPosition() * -1;
    }

    public void resetEncoder(){
        motor.getEncoder().setPosition(0);
    }

    

}