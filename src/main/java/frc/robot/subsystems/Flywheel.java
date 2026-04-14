package frc.robot.subsystems;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configuration.CANDevice;
import frc.robot.state.FlyWheelSpeeds;

public class Flywheel extends SubsystemBase {
    
    protected final TalonFX lowerWheelMotor;
    
    protected final TalonFX upperWheelMotor;
    
    protected FlyWheelSpeeds wheelSpeeds;

    protected double lowerSpeed = 0.2;
    
    protected double upperSpeed = 0.2;

    public final Commands commands;

    public Flywheel() {
        
        this.lowerWheelMotor = new TalonFX(CANDevice.TURRET_LOWER_WHEEL_MOTOR_CONTROLLER.id);
        this.upperWheelMotor = new TalonFX(CANDevice.TURRET_UPPER_WHEEL_MOTOR_CONTROLLER.id);
        this.wheelSpeeds = FlyWheelSpeeds.STOPPED;
        this.commands = new Commands();
        
        this.lowerWheelMotor.getConfigurator().apply(Flywheel.getLowerWheelMotorConfig());
        this.upperWheelMotor.getConfigurator().apply(Flywheel.getUpperWheelMotorConfig());
        
        SmartDashboard.putData("Flywheel", this);
        
    }

    protected static TalonFXConfiguration getLowerWheelMotorConfig() {
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        config.Slot0.kS = 0.2;
        // 1rps @ 0.25v
        // 2rps @ 0.36v
        // 3rps @ 0.46v
        config.Slot0.kV = 0.1;
//        config.Slot0.kA = 0.01;
        config.Slot0.kP = 0.5;
        
        config.MotionMagic.MotionMagicAcceleration = 400;
//        config.MotionMagic.MotionMagicJerk = 4000;

//    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        
        return config;
    }
    
    protected static TalonFXConfiguration getUpperWheelMotorConfig() {
        
        TalonFXConfiguration config = Flywheel.getLowerWheelMotorConfig();
        
        return config;
        
    }

    public FlyWheelSpeeds getActualWheelSpeeds() {
        
        return FlyWheelSpeeds.fromDynamicAngularMotorShaftVelocities(
            this.lowerWheelMotor.getVelocity().asSupplier(),
            this.upperWheelMotor.getVelocity().asSupplier()
        );
        
    }
    
    public FlyWheelSpeeds getWheelSpeedsSetpoints() {
        
        return this.wheelSpeeds;
        
    }
    
    public void setWheelSpeeds(FlyWheelSpeeds wheelSpeeds) {
        
        this.wheelSpeeds = wheelSpeeds;
        
        this.lowerWheelMotor.setControl(new MotionMagicVelocityVoltage(
            this.wheelSpeeds.getLowerWheelMotorShaftAngularVelocity()
        ));
        
        this.upperWheelMotor.setControl(new MotionMagicVelocityVoltage(
            this.wheelSpeeds.getUpperWheelMotorShaftAngularVelocity()
        ));
        
    }
    
    public void stopWheels() {
        
        this.lowerWheelMotor.stopMotor();
        this.upperWheelMotor.stopMotor();
        
    }

    public void setDutyCycle() {
        this.lowerWheelMotor.set(-lowerSpeed);
        this.upperWheelMotor.set(-upperSpeed);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty(
            "Flywheel Speeds/Flywheel Lower Wheel Speed (RPS)",
            () -> this.getActualWheelSpeeds().getLowerWheelAngularVelocity().in(RotationsPerSecond),
            (double rotationsPerSecond) -> this.setWheelSpeeds(new FlyWheelSpeeds(
                () -> RotationsPerSecond.of(rotationsPerSecond),
                this.getWheelSpeedsSetpoints()::getUpperWheelAngularVelocity
            ))
        );
        
        builder.addDoubleProperty(
            "Flywheel Speeds/Flywheel Upper Wheel Speed (RPS)",
            () -> this.getActualWheelSpeeds().getUpperWheelAngularVelocity().in(RotationsPerSecond),
            (double rotationsPerSecond) -> this.setWheelSpeeds(new FlyWheelSpeeds(
                this.getWheelSpeedsSetpoints()::getLowerWheelAngularVelocity,
                () -> RotationsPerSecond.of(rotationsPerSecond)
            ))
        );
        
        builder.addDoubleProperty(
            "Flywheel Setpoints/Flywheel Lower Wheel Speed Setpoint (RPS)",
            () -> this.getWheelSpeedsSetpoints().getLowerWheelAngularVelocity().in(RotationsPerSecond),
            (double rotationsPerSecond) -> this.setWheelSpeeds(new FlyWheelSpeeds(
                () -> RotationsPerSecond.of(rotationsPerSecond),
                this.getWheelSpeedsSetpoints()::getUpperWheelAngularVelocity
            ))
        );
        
        builder.addDoubleProperty(
            "Flywheel Setpoints/Flywheel Upper Wheel Speed Setpoint (RPS)",
            () -> this.getWheelSpeedsSetpoints().getUpperWheelAngularVelocity().in(RotationsPerSecond),
            (double rotationsPerSecond) -> this.setWheelSpeeds(new FlyWheelSpeeds(
                this.getWheelSpeedsSetpoints()::getLowerWheelAngularVelocity,
                () -> RotationsPerSecond.of(rotationsPerSecond)
            ))
        );
        
        builder.addDoubleProperty(
            "Flywheel Speeds/Flywheel Lower Wheel Surface Speed (FPS)",
            () -> this.getActualWheelSpeeds().getLowerWheelSurfaceSpeed().in(FeetPerSecond),
            (double feetPerSecond) -> this.setWheelSpeeds(FlyWheelSpeeds.fromStaticWheelSurfaceVelocities(
                FeetPerSecond.of(feetPerSecond),
                this.getWheelSpeedsSetpoints().getUpperWheelSurfaceSpeed()
            ))
        );
        
        builder.addDoubleProperty(
            "Flywheel Speeds/Flywheel Upper Wheel Surface Speed (FPS)",
            () -> this.getActualWheelSpeeds().getUpperWheelSurfaceSpeed().in(FeetPerSecond),
            (double feetPerSecond) -> this.setWheelSpeeds(FlyWheelSpeeds.fromStaticWheelSurfaceVelocities(
                this.getWheelSpeedsSetpoints().getLowerWheelSurfaceSpeed(),
                FeetPerSecond.of(feetPerSecond)
            ))
        );

        builder.addDoubleProperty(
            "Flywheel Speeds/Flywheel Lower Wheel Speed",
            () -> this.lowerSpeed, 
            (double d) -> this.lowerSpeed = d
        );

        builder.addDoubleProperty(
            "Flywheel Speeds/Flywheel Upper Wheel Speed",
            () -> this.upperSpeed, 
            (double d) -> this.upperSpeed = d
        );
    }

    public class Commands {
        
        public Command shoot(FlyWheelSpeeds wheelSpeeds) {
            
            return Flywheel.this.startEnd(
                () -> Flywheel.this.setWheelSpeeds(wheelSpeeds),
				Flywheel.this::stopWheels
            );
            
        }

    }

}
