package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    public static final CANBus kCANBus = new CANBus("CANivore", "./logs/example.hoot");

    private final TalonFX leftArm = new TalonFX(30, kCANBus);
    private final TalonFX rightArm = new TalonFX(31, kCANBus);

    private final MotionMagicDutyCycle motionMagic = new MotionMagicDutyCycle(0);

    public enum armStates {
        INTAKE,
        STOW,
        IDLE
    }

    private armStates currentState = armStates.IDLE;

    private double targetPos = 0;

    public Arm() {

        TalonFXConfiguration config = new TalonFXConfiguration();

        // Motor behavior
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Motion Magic motion profile
        config.MotionMagic.MotionMagicCruiseVelocity = 140;
        config.MotionMagic.MotionMagicAcceleration = 160;

        // PID values inside the Talon
        config.Slot0.kP = 0.1;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;

        // gravity feedforward
        config.Slot0.kG = 0.05;

        //soft limits 
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 70;

        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

        leftArm.getConfigurator().apply(config);
        rightArm.getConfigurator().apply(config);

        // Make the right motor follow the left
        rightArm.setControl(new Follower(leftArm.getDeviceID(), true));
    }

    public void setState(armStates newState) {

        currentState = newState;

        switch (newState) {

            case INTAKE:
                targetPos = 69; // 67 
                break;

            case STOW:
                targetPos = 0; // 0 is ideal
                break;

            case IDLE:
                targetPos = 58; // 45
                break;
        }
    }

    public double getArmPosition() {
        return leftArm.getPosition().getValueAsDouble();
    }

    public armStates getCurrentState() {
        return currentState;
    }

    @Override
    public void periodic() {

        // Motion Magic position control
        leftArm.setControl(motionMagic.withPosition(targetPos));

        SmartDashboard.putNumber("Arm Encoder", leftArm.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm Target", targetPos);
        SmartDashboard.putString("Arm State", currentState.toString());

    }
}