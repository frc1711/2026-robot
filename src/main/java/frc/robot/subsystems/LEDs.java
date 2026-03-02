package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    private final AddressableLED leds;
    private final AddressableLEDBuffer ledBuffer;

    private final Distance metersPerLed = Meters.of(1 / 60.0);

    public LEDs(int pwmPort, int ledCount) {
        this.leds = new AddressableLED(pwmPort);
        this.ledBuffer = new AddressableLEDBuffer(ledCount);
        
        this.leds.setLength(this.ledBuffer.getLength());
    }

    // Simple patterns

    public LEDPattern green() {
        LEDPattern green = LEDPattern.solid(Color.kForestGreen);

        return green;
    }

    // Complex Patterns

    public LEDPattern decreaseYellow() {
        LEDPattern yellow = LEDPattern.steps(Map.of(0, Color.kYellow, 0.25, Color.kBlack));
        LEDPattern scrollingYellow = yellow.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), metersPerLed);

        return scrollingYellow;
    }

    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(ledBuffer));
    }
}
