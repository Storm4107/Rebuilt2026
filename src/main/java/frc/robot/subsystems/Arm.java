package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{

    public static final CANBus kCANBus = new CANBus("CANivore", "./logs/example.hoot");

    private final TalonFX leftArm = new TalonFX(30, kCANBus);
    private final TalonFX rightArm = new TalonFX(31, kCANBus);

    public enum armStates {
        INTAKE,
        STOW,
        IDLE
    }

    private armStates currentState = armStates.IDLE;

    private final PIDController pid = new PIDController(.1,0,0);

    private double targetPos = 0;

        public void setState(armStates newState) {
            if (newState != currentState) {
                pid.reset();
            }

            currentState = newState;

            switch (newState) {
                case INTAKE:
                //enter value from the encoders ()
                targetPos = 66;
                    break;

                case STOW:
                //enter value from the encoders ()
                targetPos = 0;
                break;

                case IDLE:
                //enter value from the encoders ()
                targetPos = 0;
                break;
            }
        }

        public Arm() {

            TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    // Brake mode so the arm holds position
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Inversion
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Current limit
    leftConfig.CurrentLimits.SupplyCurrentLimit = 40;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    rightConfig.CurrentLimits.SupplyCurrentLimit = 40;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    // Apply configs
    leftArm.getConfigurator().apply(leftConfig);
    rightArm.getConfigurator().apply(rightConfig);
        }

        public armStates getCurrentState() {
            return currentState;
        }

        public double getArmPos() {
            return rightArm.getPosition().getValueAsDouble();
        }

        @Override
        public void periodic() {
            double position = rightArm.getPosition().getValueAsDouble();
            double power = pid.calculate(position, targetPos);
            pid.setTolerance(1);

            power = MathUtil.clamp(power, -1, 1);

            leftArm.setControl(new DutyCycleOut(power));
            rightArm.setControl(new DutyCycleOut(power));

            SmartDashboard.putNumber("Arm Encoder", position);
            SmartDashboard.putNumber("Arm Target", targetPos);
            SmartDashboard.putString("Arm State", currentState.toString());
        }

}
