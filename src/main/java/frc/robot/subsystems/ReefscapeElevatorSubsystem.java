package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Value;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.DigitalInput;
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
    private boolean invertedMotor = true;
    private double motorSpeed = 1.0;

    //limit switch settings
    private int limitSwitchDIOportNumber = 0;
    private DigitalInput limitSwitch = new DigitalInput(limitSwitchDIOportNumber);
    

    //PID settings
    private int level = 1;
    private PIDController pid;
    private final double p = 0.1;
    private final double i = 0.0;
    private final double d = 0.01;
    private final double pidErrorTolerance = 0.2;

    //Elevator Height Presets
    private final double heightL4 = 180;
    private final double heightL3 = 79;
    private final double heightL2 = 25.5;
    private final double heightL1 = 0;

    

    //setup shuffleboard
    ShuffleboardTab debugTab = Shuffleboard.getTab("Debug");
    private final ShuffleboardLayout elevatorLayout = debugTab.getLayout("Elevator", BuiltInLayouts.kList)
        .withPosition(0,0)
        .withSize(2, 2);
    GenericEntry elevatorHeightData = elevatorLayout.add("Height",0.0).getEntry();
    GenericEntry elevatorSetPointData = elevatorLayout.add("Setpoint",0.0).getEntry();
    GenericEntry elevatorOffsetData = elevatorLayout.add("Offset",0.0).getEntry();
    GenericEntry elevatorLimitSwitchData = debugTab.add("limitHit",false).withWidget("Boolean Box").getEntry();

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

    public boolean getLimitSwitchHit(){
        return limitSwitch.get();
    }


    @Override
    public void periodic(){
        
        //use our "level" value to determine where our setpoint is.
        if(level == 1){
            pid.setSetpoint(heightL1);
        }else if(level == 2){
            pid.setSetpoint(heightL2);
        }else if(level == 3){
            pid.setSetpoint(heightL3);
        }else if(level == 4){
            pid.setSetpoint(heightL4);
        }
        
        if(getLimitSwitchHit()){
            resetEncoder();
        }

        moveTowardsSetpoint();




        //update values for shuffleboard
        elevatorHeightData.setDouble(getHeight());
        elevatorSetPointData.setDouble(pid.getSetpoint());
        elevatorLimitSwitchData.setBoolean(getLimitSwitchHit());
        
        
        
        
        
    }

    private void moveTowardsSetpoint(){
        if(level == 1){
            //Separate rules for when we are trying to go all the way down
            if(getLimitSwitchHit()){
                setMotorSpeed(-0.05);
            }else{
                if(getHeight() > 10){
                    setMotorSpeed(-1.0);
                }else{
                    setMotorSpeed(-0.5);
                }
            }
        }else{
            //the limit switch is not hit, use the pid to find our setpoints
            if(pid.atSetpoint() == false){
                setMotorSpeed(pid.calculate(getHeight()));
            }else{
                setMotorSpeed(0);
            }
        }


        
    }

    private void setSetpoint(int level1to4){
        double setpoint = motor.getEncoder().getPosition();
        switch (level1to4){
            case 1:
                level = 1;
                break;
            case 2:
                level = 2;
                break;
            case 3:
                level = 3;
                break;
            case 4:
                level = 4;
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



