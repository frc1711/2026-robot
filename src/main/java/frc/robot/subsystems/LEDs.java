package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;

import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    //private final AddressableLED leds;
    //private final AddressableLEDBuffer ledBuffer;


    private final CANdle candle;

    private final Distance metersPerLed = Meters.of(1 / 60.0);
    private final int startNumber;
    private final int endNumber;

    public LEDs(int candlePort, int startCount, int endCount) {
        this.candle = new CANdle(candlePort);

        this.startNumber = startCount;
        this.endNumber = endCount;
    }

    // Simple patterns

    public SolidColor green() {
        SolidColor green = new SolidColor(startNumber, endNumber).withColor(RGBWColor.fromHex("#06402B").get());

        return green;
    }

    // Complex Patterns

    public LarsonAnimation decreaseYellow() {
        LarsonAnimation scrollingYellow = new LarsonAnimation(startNumber, endNumber).withColor(RGBWColor.fromHex("#ECA927").get()).withBounceMode(LarsonBounceValue.Front);

        return scrollingYellow;
    }

    public Command runSolid(SolidColor color) {
        return run(() -> this.candle.setControl(color));
    }
}
