package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Breach;
import frc.robot.subsystems.Conveyer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Commands.armCommand;
import frc.robot.Commands.fireCommand;
import frc.robot.Commands.intakeCommand;
import frc.robot.Commands.shootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.Commands.CommandSwerveDrivetrain;

public class RobotContainer {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain();
    public final Intake intake = new Intake();
    public final Conveyer conveyer = new Conveyer();
    public final Arm arm = new Arm();
    public final Shooter shooter = new Shooter();
    public final Breach breach = new Breach();

    // Controllers
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandGenericHID buttonBoard = new CommandGenericHID(1);

    // Auto chooser
    private SendableChooser<Command> autoChooser;

    public RobotContainer() {

        NamedCommands.registerCommand("intakeDown", new armCommand(arm, 0).withTimeout(3));
        NamedCommands.registerCommand("intakeIdle", new armCommand(arm, 2));
        NamedCommands.registerCommand("intake", new intakeCommand(intake, conveyer, breach, true).withTimeout(6));
        NamedCommands.registerCommand("Outtake", new intakeCommand(intake, conveyer, breach, false));
        NamedCommands.registerCommand("longShot", new shootCommand(shooter, 2));
        NamedCommands.registerCommand("shortShot", new shootCommand(shooter, 1).withTimeout(2));
        NamedCommands.registerCommand("shooterIdle", new shootCommand(shooter, 0));
        NamedCommands.registerCommand("fire", new fireCommand(intake, conveyer, breach));

    
    
        configureBindings();         // Button mappings
        initializeAutoChooser();     // PathPlanner Auto chooser
    }

    private void configureBindings() {
        // Example drivetrain default command
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRawAxis(2) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Example button bindings
        buttonBoard.button(4).whileTrue(new armCommand(arm,0));
        buttonBoard.button(2).whileTrue(new armCommand(arm,1));
        buttonBoard.pov(180).whileTrue(new shootCommand(shooter, 1));
        buttonBoard.pov(0).whileTrue(new shootCommand(shooter, 2));
        buttonBoard.pov(90).whileTrue(new shootCommand(shooter, 3));
        buttonBoard.button(1).whileTrue(new fireCommand(intake, conveyer, breach));
        buttonBoard.button(6).whileTrue(new intakeCommand(intake, conveyer, breach, true));
        buttonBoard.axisGreaterThan(3, 0).whileTrue(new intakeCommand(intake, conveyer, breach, false));

        joystick.button(16).onTrue(new InstantCommand(() -> drivetrain.resetHeading()));
    }

    private void initializeAutoChooser() {

        // Build chooser from PathPlanner JSON
        try {
            autoChooser = AutoBuilder.buildAutoChooser("TesterAuto"); // matches deploy/PathPlanner/Default Auto.pathplanner
        } catch (Exception e) {
            e.printStackTrace();
            autoChooser = new SendableChooser<>();
            autoChooser.setDefaultOption("Do Nothing", Commands.none());
        }

        // Add chooser to Shuffleboard
        Shuffleboard.getTab("Autonomous").add("Auto Chooser", autoChooser);
    }

    /** Call this from Robot.java autonomousInit() */
    public Command getAutonomousCommand() {
        if (autoChooser != null && autoChooser.getSelected() != null) {
            return autoChooser.getSelected();
        }
        return Commands.none();
    }
}