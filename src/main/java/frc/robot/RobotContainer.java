package frc.robot;

import edu.wpi.first.wpilibj2.command.*;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    /* Subsystems */
    private final Swerve swerve;
    private final Intake m_intake;
    private final Shooter m_shoot;
    private final Climber m_climb;
    private final IntermediateWheel m_intermid;
    // private final CenterAutonmous auto = new CenterAutonmous(swerve, m_shoot, m_intermid);
    
    
    /* Controllers */
    public final static Joystick driver = new Joystick(0);

//probably not useful
//     private final CommandXboxController driver2 =
//     new CommandXboxController(0);

    public final static Joystick operator = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Operator Buttons */
    // private final JoystickButton operatorStowButton = new JoystickButton(operator, XboxController.Button.kA.value);
    // private final JoystickButton shootButton = new JoystickButton(operator, XboxController.Button.kB.value);
    // private final JoystickButton intermidMotor = new JoystickButton(operator, XboxController.Button.kY.value);
    // private final JoystickButton operatorX = new JoystickButton(operator, XboxController.Button.kX.value);

    private final JoystickButton outtakeButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton reverseButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton intake = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton climberUp = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton climberDown = new JoystickButton(operator, XboxController.Button.kB.value);
    // private final JoystickButton shootAmp = new JoystickButton(operator, XboxController.Button.kY.value);
    // private final POVButton toggleAutoSubsystems = new POVButton(operator, 0);
    // private final POVButton operatorNinety = new POVButton(operator, 90);
    // private final POVButton operatorOneEighty = new POVButton(operator, 180);
    // private final POVButton operatorTwoSeventy = new POVButton(operator, 270);

    /* Driver Buttons */
    private final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driverRightBump = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton driverLeftBump = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kB.value);
    // private final JoystickButton outtakeButtonDriver = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

    // private final JoystickButton driverLimelight = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    // private final JoystickButton driverStowButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final POVButton operatorZero = new POVButton(operator, 0);
    // private final POVButton driverNinety = new POVButton(driver, 90);
    // private final POVButton driverOneEighty = new POVButton(driver, 180);
    // private final POVButton driverTwoSeventy = new POVButton(driver, 270);

    // /* LED Initialization */
    // private final DigitalOutput lightEight = new DigitalOutput(8);  
    // private final DigitalOutput lightNine = new DigitalOutput(9);

    /* Variables */
    private final EventLoop eventLoop = new EventLoop();
    private boolean autoSubsystems = false; // Disables/enables automatic subsystem functions (e.g. auto-intake)
    private final SendableChooser<Command> chooser;

    


    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */

    public RobotContainer() {
        swerve = new Swerve();
        m_intake = new Intake();
        m_shoot = new Shooter();
        m_climb = new Climber();
        m_intermid = new IntermediateWheel();

        //Registering Auto Commands for Path Finder:: MUST ALWAYS PUT COMMANDS IN HERE TO USE IN PATHFINDER!
        registerAutoCommands();
        

        chooser = AutoBuilder.buildAutoChooser("defaultAuto");
        SmartDashboard.putData("Auto Choices", chooser);

        
        swerve.setDefaultCommand(
                new TeleopSwerve(
                        swerve,
                        () -> -driver.getRawAxis(translationAxis)*.95,
                        () -> -driver.getRawAxis(strafeAxis)*.95,
                        () -> -driver.getRawAxis(rotationAxis)*.95,
                        robotCentric
                )
        );

        
        //driver.getRawAxis(strafeAxis)*.7

        // elevator.setDefaultCommand(
        //         new TeleopElevator(
        //                 elevator,
        //                 () -> -operator.getRawAxis(elevatorAxis)
        //         )
        // );

        // Configure the button bindings
        configureButtonBindings();
        
        
       
    }




    public void periodic() {
        eventLoop.poll();
    }

    public void registerAutoCommands() {
        NamedCommands.registerCommand("Shoot Piece", new ShootAutoCommand(m_shoot, m_intermid, 1));
        NamedCommands.registerCommand("Intake Piece 1", new IntakeAutoCommand(m_intake, m_intermid, 1));
        NamedCommands.registerCommand("Intake Piece 2", new IntakeAutoCommand(m_intake, m_intermid, 2));
        NamedCommands.registerCommand("Intake Piece 3", new IntakeAutoCommand(m_intake, m_intermid, 3));
        NamedCommands.registerCommand("Intake Piece", new IntakeAutoCommand(m_intake, m_intermid, 3.5));
        NamedCommands.registerCommand("Shoot Piece 3", new ShootAutoCommand(m_shoot, m_intermid, 1));
        
       
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    private void configureButtonBindings() {
        /* Driver Buttons */

        // shootAmp.whileTrue(new ShootAmp(m_shoot, m_intermid));
        outtakeButton.whileTrue(new ShootCommand(m_shoot, m_intermid));
        intake.whileTrue(new IntakeCommand(m_intake, m_intermid));
        reverseButton.whileTrue(new InstantCommand(m_intake::intakeReverse)).onFalse(new InstantCommand(m_intake::intakeOff));
        zeroGyro.onTrue(new InstantCommand(swerve::zeroGyro));
        
        // intake.onTrue(new InstantCommand(m_intake::intakeOn)).onFalse(new InstantCommand(m_intake::intakeOff));
        // shoot.onTrue(new InstantCommand(m_shoot::shooterOn)).onFalse(new InstantCommand(m_shoot::shooterOff));
        // intermid.onTrue(new InstantCommand(m_intermid::wheelOn)).onFalse(new InstantCommand(m_intermid::wheelOff));
        // intermidShoot.onTrue(new InstantCommand(m_intermid::wheelOn)).onFalse(new InstantCommand(m_intermid::wheelOff));

        climberUp.onTrue(new InstantCommand(m_climb::climberOn)).onFalse(new InstantCommand(m_climb::climberOff));
        climberDown.onTrue(new InstantCommand(m_climb::climberRetract)).onFalse(new InstantCommand(m_climb::climberOff));
        // shootAmp.onTrue(new InstantCommand(m_shoot::shooterAmp)).onFalse(new InstantCommand(m_shoot::shooterOff));

        //operatorNinety.whileTrue(new AutoShooter(shooter));
        driverA.whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        driverB.whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        driverRightBump.whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driverLeftBump.whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));


        //  BooleanEvent revShooterPressed = operator.axisGreaterThan(revShooter, Constants.TRIGGER_DEADBAND, eventLoop);
        //  revShooterPressed.ifHigh(
        //          () ->shooter.shoot(Constants.Shooter.BASE_SHOOTER_SPEED +
        //                  limelight.getRZ() * Constants.Shooter.DISTANCE_MULTIPLIER, false)
        //  );

        //  BooleanEvent revShooterNotPressed = revShooterPressed.negate();
        //  revShooterNotPressed.ifHigh(
        //          () -> shooter.shoot(autoSubsystems ? Constants.Shooter.IDLE_SHOOTER_SPEED : 0, true)
        //  );

        // operator.button(XboxController.Button.kRightBumper,)


        // driverStowButton.onTrue(
        //         new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_STOW))
        // );

        // operatorStowButton.onTrue(
        //         new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_STOW))
        // );

        // elevatorAmp.onTrue(new InstantCommand(() ->
        //         elevator.setHeight(Constants.Elevator.ELEVATOR_AMP)
        // ));
    }

    // smartdashboard prints
    public void printValues() {
        // robot position
        SmartDashboard.putString("Robot Pose2d", swerve.getPose().getTranslation().toString());
        SmartDashboard.putNumber("Robot Yaw", swerve.getYaw());
        SmartDashboard.putNumber("Mod 1 Distance", swerve.distanceMod1());
        SmartDashboard.putNumber("Mod 2 Distance", swerve.distanceMod2());
        SmartDashboard.putNumber("Mod 3 Distance", swerve.distanceMod3());
        SmartDashboard.putNumber("Mod 4 Distance", swerve.distanceMod4());
        // SmartDashboard.putNumber("Robot Pitch", swerve.getPitch());
        // SmartDashboard.putNumber("Robot Roll", swerve.getRoll());
    }

        // elevator debug
        // SmartDashboard.putNumber("Elevator Position", elevator.getPosition());
        // SmartDashboard.putNumber("Elevator Target", elevator.getTarget());
        // SmartDashboard.putNumber("Elevator Power", elevator.elevatorPower());
        // SmartDashboard.putBoolean("Elevator Limit Triggered?",
        // elevator.elevatorSwitchTriggered());

        //shooter debug

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {
        // TODO: work on paths
        // swerve.gyro.setYaw(-119);
        // swerve.zeroGyro();
        return chooser.getSelected();
        // return auto;
    }
}
