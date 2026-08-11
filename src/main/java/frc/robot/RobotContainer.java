// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.climber.ClimberRetract;
import frc.robot.commands.climber.ClimberClimb;
import frc.robot.commands.climber.ClimberExtend;
import frc.robot.commands.drivetrain.DrivetrainSetXFormation;
import frc.robot.commands.intake.IntakeReverse;
import frc.robot.commands.intake.IntakeRun;
import frc.robot.commands.shooterSystem.ShooterSystemRun;
import frc.robot.commands.shooterSystem.ShooterSystemRunReverse;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.cameraserver.CameraServer;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	public static final double GAMEPAD_AXIS_THRESHOLD = 0.15;
	public static final double JOYSTICK_AXIS_THRESHOLD = 0.15;


	//axis id
	public static final int LX = 0; //not used
	public static final int LY = 1;//not used
	public static final int LT = 2;
	public static final int RT = 3;
	public static final int RX = 4; //not used
	public static final int RY = 5;//not used

	// choosers (for auton)
	
	public static final String AUTON_DO_NOTHING = "Do Nothing";
	public static final String AUTON_SIMPLE_DRIVE_AND_SHOOT = "Simple Drive and Shoot";
	public static final String AUTON_DRIVE_PLUS = "Drive plus";
	//private String autonSelected;
	private SendableChooser<String> autonOptionChooser = new SendableChooser<>();

	// motorized devices

	private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
	private final Intake intake = new Intake();
	private final Shooter shooter = new Shooter();
	private final Agitator agitator = new Agitator();
	private final Climber climber = new Climber();

	// The driver's and copilot's joystick(s) and controller(s)

	CommandXboxController driverController = new CommandXboxController(Ports.USB.DRIVER_CONTROLLER); //RIGHT_JOYSTICK);
	CommandXboxController coDriverController = new CommandXboxController(Ports.USB.CODRIVER_CONTROLLER); //MAIN_JOYSTICK);
	CommandXboxController characterizationController = new CommandXboxController(Ports.USB.CHARACTERIZATION_CONTROLLER);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		autonOptionChooser.setDefaultOption("Drive plus", AUTON_DRIVE_PLUS);
		autonOptionChooser.addOption("Simple Drive and Shoot", AUTON_SIMPLE_DRIVE_AND_SHOOT);
		autonOptionChooser.addOption("Do nothing", AUTON_DO_NOTHING);
		SmartDashboard.putData("Auton options", autonOptionChooser);

		// Configure the button bindings

		configureButtonBindings();


		// Configure default commands

		drivetrain.setDefaultCommand(
			new RunCommand(
				() -> drivetrain.drive(
					MathUtil.applyDeadband(driverController.getLeftY(), GAMEPAD_AXIS_THRESHOLD),
					MathUtil.applyDeadband(driverController.getLeftX(), GAMEPAD_AXIS_THRESHOLD),
					-MathUtil.applyDeadband(driverController.getRightX(), GAMEPAD_AXIS_THRESHOLD),
					true, false),
				drivetrain));

		CameraServer.startAutomaticCapture();
		
		// publish camera stream to Shuffleboard "Vision" tab
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {

		// driver controls
		driverController.axisGreaterThan(LT,0.5)
			.whileTrue(new ShooterSystemRun(agitator, shooter));

		driverController.axisGreaterThan(RT,0.5)
			.whileTrue(new IntakeRun(intake));

		driverController.x()
			.whileTrue(new DrivetrainSetXFormation(drivetrain));

		driverController.y()
			.whileTrue(new IntakeReverse(intake));
			
		driverController.b()
			.whileTrue(new ShooterSystemRunReverse(agitator, shooter));

		driverController.rightBumper()
			.whileTrue(new ClimberRetract(climber));

		driverController.leftBumper()
			.whileTrue(new ClimberExtend(climber));

		driverController.rightStick().onTrue(new InstantCommand(()->{
			drivetrain.MaxSpeedMultiplier = Constants.DrivetrainConstants.MAX_SPEED_IN_TURBO_MODE_MULTIPLIER;
		})).onFalse(new InstantCommand(()->{
			drivetrain.MaxSpeedMultiplier = 1.0;
		}));
			
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		String autonOption = autonOptionChooser.getSelected();
		System.out.println("Auton option: " + autonOption);
		
		switch (autonOption) {
			case AUTON_SIMPLE_DRIVE_AND_SHOOT:
				return new RunCommand(
					() -> drivetrain.drive(0.28, 0, 0, false, false),
					drivetrain)
					.withTimeout(3.5).andThen(new InstantCommand(()->drivetrain.stop(),drivetrain)).andThen(new ShooterSystemRun(agitator, shooter).repeatedly().withTimeout(5.0));
			
			case AUTON_DRIVE_PLUS:
				return new RunCommand(
					() -> drivetrain.drive(0.6/*0.28*/, 0, 0, false, false),
					drivetrain)
					.withTimeout(((3.5*0.28)/0.6)).andThen(new InstantCommand(()->drivetrain.stop(),drivetrain)).andThen(new ShooterSystemRun(agitator, shooter).repeatedly().withTimeout(3.5)).andThen(new RunCommand(
					() -> drivetrain.drive(0.0, 0.3, -0.24/*-0.17 */, false, false),
					drivetrain).withTimeout(2.5).andThen(new InstantCommand(()->drivetrain.stop(),drivetrain))).andThen(new RunCommand(
					() -> drivetrain.drive(0.3, 0.0, 0.00, false, false),
					drivetrain).withTimeout(1.5/*1.7*/).andThen(new InstantCommand(()->drivetrain.stop(),drivetrain))).andThen(new ClimberExtend(climber)).andThen(new RunCommand(
					() -> drivetrain.drive(0.0, 0.0/*0.0 */, -0.28, true, false),
					drivetrain).withTimeout(1.0/*1.4*/).andThen(new InstantCommand(()->drivetrain.stop(),drivetrain))).andThen(new RunCommand(
					() -> drivetrain.drive(0.0, 0.0, 0.45, false, false),
					drivetrain).withTimeout(1.4/*1.4*/).andThen(new InstantCommand(()->drivetrain.stop(),drivetrain))).andThen(new ClimberClimb(climber));
			
			case AUTON_DO_NOTHING:
			default:
				return null;
		} 
	}

}
