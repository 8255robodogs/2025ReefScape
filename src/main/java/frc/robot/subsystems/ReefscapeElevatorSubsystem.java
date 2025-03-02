package frc.robot.subsystems;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReefscapeElevatorSubsystem extends SubsystemBase{
    

    //Other settings
    private double manualUpSpeed = 0.32;
    private double manualDownSpeed = -0.35; // MUST BE NEGATIVE TO GO DOWNWARD
    private XboxController xbox1 = new XboxController(1);

    //Motor settings
    private SparkMax motor;
    private int motorControllerCanID = 9;

    //PID settings
    private PIDController pid;
    private final double p = 0.5;
    private final double i = 0.001;
    private final double d = 0.1;
    private final double pidErrorTolerance = 0.05;

    //Elevator Height Presets
    private final double heightForTopPole = 59.9;
    private final double heightForMiddlePole = 25;
    private final double heightForBottomPole = 10;
    private final double heightForPickup = 0;
    
    //Constructor
    public ReefscapeElevatorSubsystem(){
        motor = new SparkMax(motorControllerCanID,MotorType.kBrushless);
        pid = new PIDController(p, i, d);
        pid.setSetpoint(0);
        pid.setTolerance(pidErrorTolerance);
    }
    


    private void setMotorSpeed(double speed){
        motor.set(speed * -1);
    }

    private double getHeight(){
        return motor.getEncoder().getPosition() *-1;
    }



    @Override
    public void periodic(){
        if(xbox1.getLeftTriggerAxis() < 0.5){
            //normal operation
            setMotorSpeed(pid.calculate(getHeight()));
        }else{
            //we are in manual mode
            if(xbox1.getAButton()){
                setMotorSpeed(manualDownSpeed);
            }else if(xbox1.getYButton()){
                setMotorSpeed(manualUpSpeed);
            }else{
                setMotorSpeed(0);
            }
        }
    }

    /** Set the elevator's desired height to the Top Pole's height */
    public Command goToTopPoleHeight() {
        return this.runOnce(() -> pid.setSetpoint(heightForTopPole));
    }

    /** Set the elevator's desired height to the Middle Pole's height */
    public Command goToMiddlePoleHeight() {
        return this.runOnce(() -> pid.setSetpoint(heightForMiddlePole));
    }

    /** Set the elevator's desired height to the Bottom Pole's height */
    public Command goToBottomPoleHeight() {
        return this.runOnce(() -> pid.setSetpoint(heightForBottomPole));
    }

    /** Set the elevator's desired height to the pickup height */
    public Command goToPickupHeight() {
        return this.runOnce(() -> pid.setSetpoint(heightForPickup));
    }

    

}



