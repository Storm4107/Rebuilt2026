package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyer extends SubsystemBase{

    public static final CANBus kCANBus = new CANBus("CANivore", "./logs/example.hoot");
    private final TalonFX conveyer = new TalonFX(33, kCANBus);

    //decelaring the Intake States
    public enum conveyerStates {INTAKE, IDLE, REVERSE}
    // sets the current (default) state of the intake
    private conveyerStates currentState = conveyerStates.IDLE;

    public void setState(conveyerStates newState) {
        currentState = newState;
    }

    public conveyerStates getCurrentState(){
        return currentState;
    }

    @Override
    public void periodic() {
        switch (currentState) {
            case INTAKE:
                conveyer.setControl(new DutyCycleOut(-.2));
                break;

            case REVERSE:
            conveyer.setControl(new DutyCycleOut(.2));
                break;

            case IDLE:
            conveyer.setControl(new DutyCycleOut(0));
                break;
        }
    }
}
