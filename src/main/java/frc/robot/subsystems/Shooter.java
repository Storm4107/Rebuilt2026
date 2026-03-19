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
    private final TalonFX hood = new TalonFX(44, kCANBus);

    private final MotionMagicDutyCycle motionMagic = new MotionMagicDutyCycle(0);

    public enum shooterStates {
        IDLE,
        SHORTSHOT,
        LONGSHOT,
        PASS
    }

    private shooterStates currentState = shooterStates.IDLE;

    private final PIDController pid = new PIDController(0.03,0,0); // shooter PID

    private double targetVelocity = 0;

    private double targetPos = 0;

    private static final double HOOD_TOLERANCE = 0.1;

    public Shooter() {

        TalonFXConfiguration shootConfig = new TalonFXConfiguration(); // creating shooterConfigs

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration(); // creating hoodConfigs

        //shooter Configs

        shootConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        //hood Configs 

        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 280;
        hoodConfig.MotionMagic.MotionMagicAcceleration = 210;

        hoodConfig.Slot0.kP = .1;
        hoodConfig.Slot0.kI = 0.0;
        hoodConfig.Slot0.kD = 0.0;
        hoodConfig.Slot0.kG = 0.05;

        hoodConfig.CurrentLimits.SupplyCurrentLimit = 30;

        /*hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = -.2;

        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -1;*/

        leftShooter.getConfigurator().apply(shootConfig);
        rightShooter.getConfigurator().apply(shootConfig);

        hood.getConfigurator().apply(hoodConfig);

        hood.setPosition(0);

        rightShooter.setControl(new Follower(leftShooter.getDeviceID(), true));
    }

    public void setState(shooterStates newState) {
        if (newState != currentState) {
            pid.reset();
        }

        currentState = newState;

        switch (newState) {
            case IDLE:
            targetVelocity = 0;
            targetPos = -0.02;
            break;

            case SHORTSHOT:
            targetVelocity = 40; // is 30
            //targetPos = .2;
            break;

            case LONGSHOT:
            targetVelocity = 50;
            targetPos = .3;
            break;

            case PASS:
            targetVelocity = 120;
            targetPos = .7;
            break;
        }

        pid.setSetpoint(targetVelocity);
    }

    public boolean hoodAtTarget() {
        double error = Math.abs(getHoodPos() - targetPos);
        return error < HOOD_TOLERANCE;
    }

    public shooterStates getCurrentState() {
        return currentState;
    }

    public double getShooterVelocity() {
        return leftShooter.getVelocity().getValueAsDouble();
    }

    public boolean atSpeed() {
        return pid.atSetpoint();
    }

    public double getHoodPos() {
        return hood.getPosition().getValueAsDouble();
    }

    @Override 
    public void periodic() {

        double hoodError = targetPos - hood.getPosition().getValueAsDouble();
        
        double velocity = leftShooter.getVelocity().getValueAsDouble();
        double power = pid.calculate(velocity, targetVelocity);

        switch (currentState) {
            case IDLE:
            leftShooter.setControl(new DutyCycleOut(power));
            hood.setControl(motionMagic.withPosition(targetPos));
                break;

            case SHORTSHOT:
            leftShooter.setControl(new DutyCycleOut(power));
            hood.setControl(motionMagic.withPosition(targetPos));
            break;

            case LONGSHOT:
            leftShooter.setControl(new DutyCycleOut(power));
            hood.setControl(motionMagic.withPosition(targetPos));
            break;

            case PASS:
            leftShooter.setControl(new DutyCycleOut(power));
            hood.setControl(motionMagic.withPosition(targetPos));
            break;
        }

            leftShooter.setControl(new DutyCycleOut(power));
            hood.setControl(motionMagic.withPosition(targetPos));

        SmartDashboard.putNumber("hood Encoder", hood.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("hood Target", targetPos);

        SmartDashboard.putNumber("Shooter RPS", leftShooter.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("ShooterTarget", targetVelocity);
        SmartDashboard.putString("Shooter State", currentState.toString());
    }
}