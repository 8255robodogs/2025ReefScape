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
    private int numPixels = 60;    // The number of addressable LEDs in the strip
    private AddressableLEDBuffer buffer;

    LEDPattern rainbow = LEDPattern.rainbow(255,128);
    Distance kLedSpacing = Meters.of(1 / 120.0);
    private final LEDPattern scrollingRainbow =
        rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

    // Constructor to initialize the LED strip
    public ReefscapeLEDSubsystem() {
        this.ledStrip = new AddressableLED(ledStripPWMportNumber);
        ledStrip.setLength(numPixels);
        buffer = new AddressableLEDBuffer(numPixels);

        scrollingRainbow.applyTo(buffer);
        ledStrip.setData(buffer);
    }


    @Override
    public void periodic() {
        scrollingRainbow.applyTo(buffer);
        ledStrip.setData(buffer);
    }
}
