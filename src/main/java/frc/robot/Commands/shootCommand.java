package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.shooterStates;
import frc.robot.subsystems.Shooter.shooterStates;

public class shootCommand extends Command {

    private final Shooter shooter;
    private final int preset;

    // Array mapping preset numbers to states
    private final shooterStates[] presets = {
        shooterStates.IDLE,
        shooterStates.SHORTSHOT,
        shooterStates.LONGSHOT,
        shooterStates.PASS
    };

    public shootCommand(Shooter shooterSub, int preset) {
        this.shooter = shooterSub;
        this.preset = preset;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {

        // Check that preset index exists
        if (preset >= 0 && preset < presets.length) {
            shooter.setState(presets[preset]);
        }
        else {
            System.out.println("Invalid shooter preset: " + preset);
        }

    }

    @Override
    public void end(boolean interrupted) {
            shooter.setState(shooterStates.IDLE);
    }
}