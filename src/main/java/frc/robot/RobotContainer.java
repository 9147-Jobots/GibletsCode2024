package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Routines.AlignShooterAngle;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController xController;
    private final XboxController logitechController;
    private final Joystick driver;

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Subsystems */
    private final Mechanism s_Mechanism;
    private final Lights lights;
    private final VisionSubsystem limelight;
    private final PivotSubsystem s_Pivot;
    private final Climber s_Climber;

    private final boolean robotCentric = false;

    /* Auto Chooser */
    private SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Mechanism = new Mechanism();
        limelight = new VisionSubsystem();
        s_Pivot = new PivotSubsystem();
        lights = new Lights();
        s_Climber = new Climber();
        
        /* Constrollers */
        xController = new XboxController(0);
        driver = new Joystick(0);

        logitechController = new XboxController(1);

        JoystickButton TriggerRight = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

        s_Pivot.setDefaultCommand(
           new TeleopPivot(
            s_Pivot, 
            () -> logitechController.getLeftY(), 
            () -> logitechController.getRightBumper()
            )
        );

        s_Mechanism.setDefaultCommand(
            new TeleopMechanism(
                s_Mechanism, 
                () -> logitechController.getLeftTriggerAxis(), 
                () -> logitechController.getRightTriggerAxis(),
                () -> logitechController.getLeftBumper()
                )
        );

        s_Climber.setDefaultCommand(
            new TeleopClimber(
                s_Climber, 
                logitechController
                )
        );

        //Register named commands for auto
        NamedCommands.registerCommand("Pickup Note", new PickupNote(s_Mechanism, s_Pivot, TriggerRight));
        NamedCommands.registerCommand("Auto Shoot", new AutoShootingStationary(s_Mechanism, s_Swerve, s_Pivot));
        autoChooser = AutoBuilder.buildAutoChooser("leave from source");
        SmartDashboard.putData(autoChooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Define driver buttons */
        final Trigger buttonA = new JoystickButton(xController, 1);
        final Trigger buttonB = new JoystickButton(xController, 2);
        final Trigger buttonX = new JoystickButton(xController, 3);
        final Trigger buttonY = new JoystickButton(xController, 4);

        final Trigger TriggerLeft = new  JoystickButton(xController, XboxController.Button.kLeftBumper.value);

        final Trigger coDriverButtonA = new JoystickButton(logitechController, 1);
        final Trigger coDriverButtonB = new JoystickButton(logitechController, 2);
        final Trigger coDriverButtonX = new JoystickButton(logitechController, 3);
        final Trigger coDriverButtonY = new JoystickButton(logitechController, 4);
        final Trigger coDriverTriggerRight = new JoystickButton(logitechController, XboxController.Button.kRightBumper.value);


        JoystickButton kY = new JoystickButton(driver, XboxController.Button.kY.value);

        /* Driver Buttons */
        kY.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        //buttonX.onTrue(new AutoShootingStationary(s_Mechanism, s_Swerve, s_Pivot)); // auto stops after 3 seconds
        //buttonB.onTrue(new ReverseIntake(s_Mechanism)); // B to stop
        buttonA.onTrue(new PickupNote(s_Mechanism, s_Pivot, TriggerLeft).raceWith(new FlashLED(255,110,0,false,lights)));

        buttonY.onTrue(new InstantCommand(() -> s_Pivot.setStartAngle()));
        
        coDriverButtonA.onTrue(new InstantCommand(() -> s_Pivot.setUnderStageAngle()));
        coDriverButtonX.onTrue(new InstantCommand(() -> s_Pivot.setSafetyAngle()));
        coDriverButtonB.onTrue(new InstantCommand(() -> s_Pivot.setAmpLoadingAngle()));
        coDriverButtonY.whileTrue(new ReverseIntake(s_Mechanism));
        

        coDriverTriggerRight.whileTrue(new InstantCommand(() -> AlignShooterAngle.execute(s_Pivot)));
    }

    public String getAutoName() {
        return autoChooser.getSelected().getName();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //set the starting Pose2d here
        return autoChooser.getSelected();
        //return AutoBuilder.buildAuto("Auto 1");
    }
}
