package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    public static final CANBus kCANBus = new CANBus("CANivore", "./logs/example.hoot");
    private final TalonFX intake = new TalonFX(34, kCANBus);

    //decelaring the Intake States
    public enum intakeStates {INTAKE, IDLE, REVERSE, FIRE}
    // sets the current (default) state of the intake
    private intakeStates currentState = intakeStates.IDLE;

    public void setState(intakeStates newState) {
        currentState = newState;
    }

    public intakeStates getCurrentState(){
        return currentState;
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case INTAKE:
                intake.setControl(new DutyCycleOut(-.3));
                break;

            case REVERSE:
            intake.setControl(new DutyCycleOut(.2));
                break;

            case FIRE:
            intake.setControl(new DutyCycleOut(-.6));
                break;

            case IDLE:
            intake.setControl(new DutyCycleOut(0));
                break;
        }
    }
}
