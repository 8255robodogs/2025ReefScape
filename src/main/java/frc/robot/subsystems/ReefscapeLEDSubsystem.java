package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ReefscapeLEDSubsystem extends SubsystemBase {
    
    private AddressableLED ledStrip;  
    private int ledStripPWMportNumber = 1;
    private int numPixels = 256;    // The number of addressable LEDs in the strip
    private AddressableLEDBuffer buffer;

    // Constructor to initialize the LED strip
    public ReefscapeLEDSubsystem() {
        this.ledStrip = new AddressableLED(ledStripPWMportNumber);
        this.ledStrip.setLength(numPixels);
        this.buffer = new AddressableLEDBuffer(numPixels);

        for(int i=0;i<buffer.getLength();i++){
            buffer.setRGB(i, 180, 0, 0);
        }

        this.ledStrip.setData(buffer);

    }


    @Override
    public void periodic() {
        ledStrip.setData(buffer);
    }

}
