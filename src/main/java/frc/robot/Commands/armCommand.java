package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.armStates;

public class armCommand extends Command {

    private final Arm arm;
    private final int preset;

    // Array mapping preset numbers to states
    private final armStates[] presets = {
        armStates.INTAKE,
        armStates.STOW,
        armStates.IDLE
    };

    public armCommand(Arm armSub, int preset) {
        this.arm = armSub;
        this.preset = preset;

        addRequirements(arm);
    }

    @Override
    public void initialize() {

        // Check that preset index exists
        if (preset >= 0 && preset < presets.length) {
            arm.setState(presets[preset]);
        }
        else {
            System.out.println("Invalid arm preset: " + preset);
        }

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            arm.setState(armStates.IDLE);
        }
    }
}