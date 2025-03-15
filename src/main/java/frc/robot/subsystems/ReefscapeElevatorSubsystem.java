package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Value;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReefscapeElevatorSubsystem extends SubsystemBase{

    //Motor settings
    private SparkMax motor;
    private int motorControllerCanID = 9;
    private boolean invertedMotor = false;
    private double motorSpeed = 1.0;

    //PID settings
    private PIDController pid;
    private final double p = 0.1;
    private final double i = 0.0;
    private final double d = 0.01;
    private final double pidErrorTolerance = 1;

    //Elevator Height Presets
    private final double heightL4 = 180;
    private final double heightL3 = 79;
    private final double heightL2 = 25.5;
    private final double heightL1 = 0;
    
    //Constructor
    public ReefscapeElevatorSubsystem(){
        motor = new SparkMax(motorControllerCanID,MotorType.kBrushless);
        pid = new PIDController(p, i, d);
        pid.setSetpoint(heightL1);
        pid.setTolerance(pidErrorTolerance);
    }

    private void setMotorSpeed(double speed){
        double newSpeed = speed;
        if(invertedMotor == true){
            newSpeed = newSpeed *-1;
        }
        motor.set(newSpeed);
    }

    private double getHeight(){
        double heightToReturn = motor.getEncoder().getPosition();
        if(invertedMotor){
            heightToReturn = heightToReturn * -1;
        }
        return heightToReturn;
    }

    private void resetEncoder(){
        motor.getEncoder().setPosition(0);
    }



    @Override
    public void periodic(){
        System.out.println("elevatorHeight: " + getHeight()+ " ...... " + "elevatorSetPoint: "+ pid.getSetpoint());
        moveTowardsSetpoint();
    }

    private void moveTowardsSetpoint(){
        if(pid.atSetpoint() == false){
            setMotorSpeed(pid.calculate(getHeight()));
        }else{
            setMotorSpeed(0);
        }
    }

    private void setSetpoint(int level1to4){
        double setpoint = motor.getEncoder().getPosition();
        switch (level1to4){
            case 1:
                setpoint = heightL1;
                break;
            case 2:
                setpoint = heightL2;
                break;
            case 3:
                setpoint = heightL3;
                break;
            case 4:
                setpoint = heightL4;
                break;
            default:
                System.out.println("Invalid level!");
                break;
        }
        pid.setSetpoint(setpoint);
    }

    public Command setSetpointCommand(int level1to4){
        return Commands.runOnce(() -> setSetpoint(level1to4));
    }
    

}



