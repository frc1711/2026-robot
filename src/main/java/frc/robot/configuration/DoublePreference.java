package frc.robot.configuration;

import edu.wpi.first.wpilibj.Preferences;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * A collection of floating point preferences that can be set and retrieved
 * from the RoboRIO across power cycles.
 */
public enum DoublePreference implements DoubleSupplier {
    
    FRONT_LEFT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES("Front Left Swerve Module Encoder Offset", 0),
    
    FRONT_RIGHT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES("Front Right Swerve Module Encoder Offset", 0),
    
    REAR_LEFT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES("Rear Left Swerve Module Encoder Offset", 0),
    
    REAR_RIGHT_SWERVE_MODULE_ENCODER_OFFSET_DEGREES("Rear Right Swerve Module Encoder Offset", 0),

    JOYSTICK_DEADBAND("Joystick Deadband", 0.05),

    LINEAR_INPUT_POWER_SMOOTHING("Linear Input Power Smoothing", 3);
    
    /**
     * The key of the preference.
     */
    private final String key;
    
    /**
     * The default value of the preference to use if it is not already present
     * on the RoboRIO.
     */
    private final double defaultValue;
    
    /**
     * A list of listeners to notify when the value of this preference changes.
     */
    private final List<DoubleConsumer> listeners;
    
    /**
     * Initializes a new double preference with the given key and default value.
     *
     * @param key The key of the preference.
     * @param defaultValue The default value of the preference to use if it is
     * not already present on the RoboRIO.
     */
    DoublePreference(String key, double defaultValue) {
        
        this.key = key;
        this.defaultValue = defaultValue;
        this.listeners = new ArrayList<>();
        
    }
    
    /**
     * Initializes all double preferences.
     */
    public static void init() {
        
        for (DoublePreference preference: DoublePreference.values()) {
            
            Preferences.initDouble(preference.key, preference.defaultValue);
            
        }
        
    }
    
    /**
     * Returns the value of the preference.
     *
     * @return The value of the preference.
     */
    public double get() {
        
        return Preferences.getDouble(this.key, this.defaultValue);
        
    }
    
    /**
     * Sets the value of the preference.
     *
     * @param value The value to set the preference to.
     */
    public void set(double value) {
        
        Preferences.setDouble(this.key, value);
        this.listeners.forEach((listener) -> listener.accept(value));
        
    }
    
    /**
     * Registers the given listener function to handle changes to the value of
     * this preference.
     *
     * @param listener The listener function to register.
     */
    public void onChange(DoubleConsumer listener) {
        
        this.listeners.add(listener);
        
    }
    
    /**
     * Passes the value of this preference to the given consumer and registers
     * the consumer to be called whenever the value of this preference changes.
     *
     * This is a convenience method for setting the value of the preference and
     * registering a listener in one line.
     *
     * @param consumer The consumer to pass the value of this preference to.
     */
    public void useValue(DoubleConsumer consumer) {
        
        consumer.accept(this.get());
        this.onChange(consumer);
        
    }

    @Override
    public double getAsDouble() {
        
        return this.get();
        
    }
    
}
