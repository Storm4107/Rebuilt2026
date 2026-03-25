package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    public static final CANBus kCANBus = new CANBus("CANivore");

    private final TalonFX leftShooter = new TalonFX(42, kCANBus);
    private final TalonFX rightShooter = new TalonFX(43, kCANBus);

    public enum shooterStates {
        IDLE,
        SHORTSHOT,
        LONGSHOT,
        PASS
    }

    private shooterStates currentState = shooterStates.IDLE;

    //private final PIDController pid = new PIDController(0.03,0,0); // shooter PID   .022

    private double targetVelocity = 0;


    public Shooter() {

        TalonFXConfiguration shootConfig = new TalonFXConfiguration(); // creating shooterConfigs

        shootConfig.Slot0.kS = 0.022;
        shootConfig.Slot0.kP = 0;

        //shooter Configs

        shootConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        leftShooter.getConfigurator().apply(shootConfig);
        rightShooter.getConfigurator().apply(shootConfig);

        rightShooter.setControl(new Follower(leftShooter.getDeviceID(), true));
    }

    public void setState(shooterStates newState) {

        currentState = newState;

        switch (newState) {
            case IDLE:
            targetVelocity = 0;
            break;

            case SHORTSHOT:
            targetVelocity = 40; // is 30
            break;

            case LONGSHOT:
            targetVelocity = 50;
            break;

            case PASS:
            targetVelocity = 120;
            break;
        }
    }



    public shooterStates getCurrentState() {
        return currentState;
    }

    public double getShooterVelocity() {
        return leftShooter.getVelocity().getValueAsDouble();
    }

    @Override 
    public void periodic() {
        
            leftShooter.setControl(new VelocityVoltage(targetVelocity));

        SmartDashboard.putNumber("Shooter RPS", leftShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("ShooterTarget", targetVelocity);
        SmartDashboard.putString("Shooter State", currentState.toString());
    }
}