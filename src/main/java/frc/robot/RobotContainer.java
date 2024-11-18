// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//Imports from WPILib
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    XboxController xboxController;
    public final IntakeSubsystem intake = new IntakeSubsystem();
    public final ElevatorSubsystem elevator= new ElevatorSubsystem();

    final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // Drivetrain
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(CommandSwerveDrivetrain.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
            .withRotationalDeadband(CommandSwerveDrivetrain.MaFxAngularRate * 0.1) 
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    public ControllerContainer controllerContainer = new ControllerContainer();
    public SendableChooser<AutoConstants.AutoMode> autoChooser;
    ButtonHelper buttonHelper = new ButtonHelper(controllerContainer.getControllers());
    private boolean autoNeedsRebuild = true;
    private Command auto;

    private final RobotCommands robotCommands = new RobotCommands(intake, drivetrain, elevator);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        xboxController = new XboxController(Constants.XBOX_CONTROLLER_PORT);

        var field = new Field2d();
        SmartDashboard.putData("Field", field);

        PathPlannerLogging.setLogCurrentPoseCallback(field::setRobotPose);

        PathPlannerLogging.setLogTargetPoseCallback(pose ->
                field.getObject("target pose").setPose(pose)
        );

        PathPlannerLogging.setLogActivePathCallback(poses ->
                field.getObject("path").setPoses(poses)
        );

        Telemetry logger = new Telemetry(CommandSwerveDrivetrain.MAX_VELOCITY_METERS_PER_SECOND);
        drivetrain.registerTelemetry(logger::telemeterize);
        
        configureBindings();

        buildAutoChooser();
        rebuildAutoIfNecessary();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () ->
                                drive
                                        // +X in velocity = forward, -Y in joystick = forward
                                        .withVelocityX(-joystick.getLeftY() * TunerConstants.kSpeedAt12VoltsMps)
                                        // +Y in velocity = left, -X in joystick = left
                                        .withVelocityY(-joystick.getLeftX() * TunerConstants.kSpeedAt12VoltsMps)
                                        // +rotational rate = counterclockwise (left), -X in joystick = left
                                        .withRotationalRate(-joystick.getRightX() * CommandSwerveDrivetrain.MaFxAngularRate)
                ));

        // Uses the intake to suck in the game pieces
        joystick.a().whileTrue(intake.rollIn(1).alongWith(intake.toggleRamp().withTimeout(1).andThen(intake.toggleRamp()));

        // Move elevator down
        joystick.y().onTrue(elevator.setPositionTo(ElevatorSubsystem.ElevatorConstants.ElevatorPositions.DOWN));

        //Move elevator up
        joystick.x().onTrue(elevator.setPositionTo(ElevatorSubsystem.ElevatorConstants.ElevatorPositions.UP));

        

        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
