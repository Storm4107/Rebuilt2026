package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.intakeStates;

public class Breach extends SubsystemBase{

    public static final CANBus kCANBus = new CANBus("CANivore", "./logs/example.hoot");
    private final TalonFX leftBreach = new TalonFX(41, kCANBus); /// also the top breach
    private final TalonFX rightBreach = new TalonFX(40, kCANBus);

    public enum BreachStates{
        IDLE,
        INTAKING,
        REVERSE,
        FIRE
    }

    private BreachStates currentState = BreachStates.IDLE;

    private static final TalonFXConfiguration leftBreachConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration rightBreachConfigs = new TalonFXConfiguration();

    public Breach() {

        leftBreachConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightBreachConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        leftBreachConfigs.CurrentLimits.SupplyCurrentLimit = 80;
        rightBreachConfigs.CurrentLimits.SupplyCurrentLimit = 80;

        leftBreach.getConfigurator().apply(leftBreachConfigs);
        rightBreach.getConfigurator().apply(rightBreachConfigs);

        leftBreachConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leftBreach.setControl(new Follower(rightBreach.getDeviceID(), false));
    }


    public void setState(BreachStates newState) {
        currentState = newState;
    }

    public BreachStates getCurrentState(){
        return currentState;
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case INTAKING:
                leftBreach.setControl(new DutyCycleOut(-.1));
                rightBreach.setControl(new DutyCycleOut(0));
                break;

            case REVERSE:
                leftBreach.setControl(new DutyCycleOut(-.4));
                rightBreach.setControl(new DutyCycleOut(-.4));
                break;

            case IDLE:
                leftBreach.setControl(new DutyCycleOut(0));
                rightBreach.setControl(new DutyCycleOut(0));
                break;

            case FIRE:
                leftBreach.setControl(new DutyCycleOut(.7));
                rightBreach.setControl(new DutyCycleOut(.7));
                break;
        }
    }
    
}
