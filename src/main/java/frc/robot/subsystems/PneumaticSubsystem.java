package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticSubsystem extends SubsystemBase{
    
    DoubleSolenoid solenoid;

    public PneumaticSubsystem(int pcmCan, int firstPort, int secondPort, boolean startInReverse){
        solenoid = new DoubleSolenoid(pcmCan,PneumaticsModuleType.CTREPCM, firstPort, secondPort);
        if(startInReverse){
            solenoid.set(Value.kReverse);
        }else{
            solenoid.set(Value.kForward);
        }
    }

    public void TogglePneumatic(){
        solenoid.toggle();
    }

    public void PneumaticForward(){
        solenoid.set(Value.kForward);
    }

    public void PneumaticReverse(){
        solenoid.set(Value.kReverse);
    }

    public DoubleSolenoid getSolenoid(){
        return this.solenoid;
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }

}