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

    public SparkMaxMotor(int canID){
        motor = new SparkMax(canID,MotorType.kBrushless);
        
    }

    public void setSpeed(double speed){
        motor.set(speed);
    }

    public double getSpeed(){
        return motor.get();
    }

}