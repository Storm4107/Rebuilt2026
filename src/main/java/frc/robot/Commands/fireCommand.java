package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Breach;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Breach.BreachStates;
import frc.robot.subsystems.Conveyer.conveyerStates;
import frc.robot.subsystems.Intake.intakeStates;

public class fireCommand extends Command{

    private final Intake intake;
    private final Conveyer conveyer;
    private final Breach breach;

    public fireCommand(Intake intakeSub, Conveyer conveyerSub, Breach breachSub){
        intake = intakeSub;
        conveyer = conveyerSub;
        breach = breachSub;
    }

    @Override
    public void execute(){

            intake.setState(Intake.intakeStates.INTAKE);
            conveyer.setState(conveyerStates.INTAKE);
            breach.setState(BreachStates.FIRE);
        
    }

    @Override
    public void end(boolean interrupted) {
        intake.setState(Intake.intakeStates.IDLE);
        conveyer.setState(conveyerStates.IDLE);
        breach.setState(BreachStates.IDLE);
    }
}
