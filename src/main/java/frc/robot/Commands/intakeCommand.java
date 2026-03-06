package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Conveyer.conveyerStates;
import frc.robot.subsystems.Intake.intakeStates;

public class intakeCommand extends Command{

    private final Intake intake;
    private final Conveyer conveyer;
    private final boolean intaking;

    public intakeCommand(Intake intakeSub, Conveyer conveyerSub, boolean intaking){
        intake = intakeSub;
        conveyer = conveyerSub;
        this.intaking = intaking;
    }

    @Override
    public void execute(){

        if (intaking) {
            intake.setState(Intake.intakeStates.INTAKE);
            conveyer.setState(conveyerStates.INTAKE);
        } else {
            intake.setState(Intake.intakeStates.REVERSE);
            conveyer.setState(conveyerStates.REVERSE);
        }

        
    }

    @Override
    public void end(boolean interrupted) {
        intake.setState(Intake.intakeStates.IDLE);
        conveyer.setState(conveyerStates.IDLE);
    }
}
