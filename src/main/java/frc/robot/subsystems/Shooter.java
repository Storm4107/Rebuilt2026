package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    public static final CANBus kCANBus = new CANBus("CANivore");

    private final TalonFX leftShooter = new TalonFX(42, kCANBus);
    private final TalonFX rightShooter = new TalonFX(43, kCANBus);
    //private final TalonFXS hood = new TalonFXS(44, kCANBus);
    private final CANcoder hoodEncoder = new CANcoder(45, kCANBus);

    // Motion Magic velocity control request
    private final MotionMagicVelocityDutyCycle velocityRequest =
        new MotionMagicVelocityDutyCycle(0);

    //private final MotionMagicDutyCycle motionMagic =  new MotionMagicDutyCycle(0);


    private static final double VELOCITY_TOLERANCE = 5; // RPS

    //private double targetPos = 0;

    private double targetVelocity = 0;

    public enum ShooterState {
        IDLE,
        SHORTSHOT,
        LONGSHOT
    }

    private ShooterState currentState = ShooterState.IDLE;

    public Shooter() {

        TalonFXConfiguration config = new TalonFXConfiguration();
        //TalonFXSConfiguration hoodConfig = new TalonFXSConfiguration();
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();

        //hoodConfig.Commutation.MotorArrangement = MotorArrangementValue.NEO550_JST;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        //hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 40;
        //hoodConfig.MotionMagic.MotionMagicAcceleration = 30;

        //hoodConfig.Slot0.kP = 1;
        //hoodConfig.Slot0.kI = 0.0;
        //hoodConfig.Slot0.kD = 0.0;

        //hoodConfig.Slot0.kG = 0.05;

        /*hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 2;

        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;*/

        //hoodConfig.ExternalFeedback.FeedbackRemoteSensorID = hoodEncoder.getDeviceID();
        //hoodConfig.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;

        // Velocity PID
        config.Slot0.kP = 0.035;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kV = 0.0;

        // Motion Magic velocity ramp settings
        config.MotionMagic.MotionMagicAcceleration = 30; // RPS per second
        config.MotionMagic.MotionMagicJerk = 0;         // change in accel

        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        //hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leftShooter.getConfigurator().apply(config);
        rightShooter.getConfigurator().apply(config);
        //hood.getConfigurator().apply(hoodConfig);
        //hoodEncoder.getConfigurator().apply(encoderConfig);

        // Right motor follows the left
        rightShooter.setControl(new Follower(leftShooter.getDeviceID(), true));
    }

    public void setState(ShooterState newState) {

        currentState = newState;

        switch (newState) {

            case IDLE:
                targetVelocity = 0;
                //targetPos = 0;
                break;

            case SHORTSHOT:
                targetVelocity = 40; // rotations per second
                //targetPos = .2;
                break;

            case LONGSHOT:
                targetVelocity = 50; // rotations per second
                //targetPos = .7;
                break;
        }
    }

    public ShooterState getState() {
        return currentState;
    }

    public double getHoodPosition() {
        return hoodEncoder.getPosition().getValueAsDouble();
    }

    public double getShooterVelocity() {
        return leftShooter.getVelocity().getValueAsDouble();
    }

    public boolean atSpeed() {
        return Math.abs(getShooterVelocity() - targetVelocity) < VELOCITY_TOLERANCE;
    }

    @Override
    public void periodic() {

        switch (currentState) {
            case IDLE:
                leftShooter.setControl(
            velocityRequest.withVelocity(targetVelocity)
            );

            //hood.setControl(motionMagic.withPosition(targetPos));

            break;

            case SHORTSHOT:
                leftShooter.setControl(
                    velocityRequest.withVelocity(targetVelocity)
            );

            //hood.setControl(motionMagic.withPosition(targetPos));

            break;

            case LONGSHOT:
                leftShooter.setControl(
                    velocityRequest.withVelocity(targetVelocity)
            );

            //hood.setControl(motionMagic.withPosition(targetPos));
            break;
        }

        

        SmartDashboard.putNumber("Shooter Velocity", getShooterVelocity());
        SmartDashboard.putNumber("Shooter Target", targetVelocity);
        SmartDashboard.putBoolean("Shooter Ready", atSpeed());

        SmartDashboard.putNumber("hood Encoder", hoodEncoder.getPosition().getValueAsDouble());
        //SmartDashboard.putNumber("hood Target", targetPos);
    }
}