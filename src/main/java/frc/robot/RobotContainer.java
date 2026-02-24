// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;

import frc.robot.sensors.*;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrivetrain;
//import frc.robot.subsystems.TelescopingArm;
//import frc.robot.subsystems.AlgaeBlaster;
//import frc.robot.subsystems.Coralator;
//import frc.robot.subsystems.Climber;
//import frc.robot.subsystems.Indicator;
//import frc.robot.commands.algae_blaster.BlastAlgae;
//import frc.robot.commands.algae_blaster.IntakeAlgae;
//import frc.robot.commands.coralator.Eject;
//import frc.robot.commands.climber.Out;
//import frc.robot.commands.climber.Climb;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.intake.*;
//import frc.robot.interfaces.ICamera;
//import frc.robot.commands.indicator.*;
//import frc.robot.commands.pivot_arm.ManuallyAdjustPivotArm;
//import frc.robot.commands.telescoping_arm.ManuallyAdjustTelescopingArm;
//import frc.robot.subsystems.PivotArm;
//import frc.robot.commands.groups.*;
import frc.robot.commands.shooterSystem.ShooterSystemRun;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

	public static final double GAMEPAD_AXIS_THRESHOLD = 0.15;
	public static final double JOYSTICK_AXIS_THRESHOLD = 0.15;

	public static final int LX = 0;
	public static final int LY = 1;
	public static final int LT = 2;
	public static final int RT = 3;
	public static final int RX = 4;
	public static final int RY = 5;

	Command indicatorTimedScrollRainbow; // command to run while stating up and when disabled

	// choosers (for auton)
	
	public static final String AUTON_DO_NOTHING = "Do Nothing";
	public static final String AUTON_CUSTOM = "My Auto";
	public static final String AUTON_SAMPLE_SWERVE = "Sample Swerve";
	public static final String AUTON_SAMPLE_MOVE_FORWARD = "Sample Move Forward";
	public static final String AUTON_SAMPLE_MOVE_IN_REVERSE = "Sample Move In Reverse";
	public static final String AUTON_SAMPLE_MOVE_IN_GAMMA_SHAPE = "Sample Move In Gamma Shape";
	public static final String AUTON_SAMPLE_MOVE_IN_L_SHAPE_IN_REVERSE = "Sample Move In L Shape In Reverse";
	public static final String AUTON_TEST_HARDCODED_MOVE_1 = "Test Hardcoded Move 1";
	public static final String AUTON_TEST_HARDCODED_MOVE_2 = "Test Hardcoded Move 2";
	public static final String AUTON_TEST_TRAJECTORY_GENERATION = "Test Trajectory Generation";
	public static final String AUTON_SIMPLE_DRIVE = "Simple Drive";
	public static final String AUTON_DRIVE_PLUS = "Drive plus";
	//private String autonSelected;
	private SendableChooser<String> autonChooser = new SendableChooser<>();

	public static final String AUTON_OPTION_JUST_SHOOT_NOTE = "Just Shoot Note";
	public static final String AUTON_OPTION_LEAVE_COMMUNITY = "Leave Community";
	public static final String AUTON_OPTION_PICKUP_NOTE_AT_MIDLINE = "Pickup Note at Midline";
	public static final String AUTON_OPTION_PICKUP_NOTE_AT_WING = "Pickup Note at Wing";
	public static final String AUTON_OPTION_FEED_NOTE = "Feed Note";
	private String autonOption;
	private SendableChooser<String> autonOptionChooser = new SendableChooser<>();

	// sensors

	private final HMAccelerometer accelerometer = new HMAccelerometer();

	//private final ICamera object_detection_camera = new ObjectDetectionCamera();

	//private final ICamera apriltag_camera = new AprilTagCamera();

	// motorized devices

	private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
	private final Intake intake = new Intake();
	private final Shooter shooter = new Shooter();
	private final Agitator agitator = new Agitator();
	/*
	private final AlgaeBlaster algaeBlaster = new AlgaeBlaster();
	private final TelescopingArm telescopingArm = new TelescopingArm();
	private final PivotArm pivotArm = new PivotArm();
	private final Coralator coralator = new Coralator();
	private final Climber climber = new Climber();
	*/

	// misc

	private final Field2d field = new Field2d(); //  a representation of the field
	//private final Indicator indicator = new Indicator(apriltag_camera, object_detection_camera);

	// The driver's and copilot's joystick(s) and controller(s)

	CommandXboxController driverController = new CommandXboxController(Ports.USB.DRIVER_CONTROLLER); //RIGHT_JOYSTICK);
	CommandXboxController coDriverController = new CommandXboxController(Ports.USB.CODRIVER_CONTROLLER); //MAIN_JOYSTICK);
	CommandXboxController characterizationController = new CommandXboxController(Ports.USB.CHARACTERIZATION_CONTROLLER);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		autonOptionChooser.setDefaultOption("Simple Drive", AUTON_SIMPLE_DRIVE);
		autonOptionChooser.addOption("Drive plus", AUTON_DRIVE_PLUS);
		autonOptionChooser.addOption("Do nothing", AUTON_DO_NOTHING);
		SmartDashboard.putData("Auton options", autonOptionChooser);

		// Configure the button bindings

		configureButtonBindings();


		// Configure default commands

		drivetrain.setDefaultCommand(
			new RunCommand(
				() -> drivetrain.drive(
					-MathUtil.applyDeadband(driverController.getLeftY(), GAMEPAD_AXIS_THRESHOLD),
					-MathUtil.applyDeadband(driverController.getLeftX(), GAMEPAD_AXIS_THRESHOLD),
					-MathUtil.applyDeadband(driverController.getRightX(), GAMEPAD_AXIS_THRESHOLD),
					true, true),
				drivetrain));

		//indicator.setDefaultCommand(new IndicatorIndicateUsingCamera(indicator)); // default command, only runs when robot is enabled

		// telescopingArm.setDefaultCommand(new ManuallyAdjustTelescopingArm(telescopingArm, coDriverController));
		// pivotArm.setDefaultCommand(new ManuallyAdjustPivotArm(pivotArm, coDriverController));
		//indicatorTimedScrollRainbow = new IndicatorTimedScrollRainbow(indicator,1);
		//indicatorTimedScrollRainbow.schedule(); // we schedule the command as we are starting up
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
		driverController.a()
			.whileTrue(new ShooterSystemRun(agitator, shooter));

		driverController.b()
			.whileTrue(new IntakeRun(intake));

		driverController.x()
			.whileTrue(new DrivetrainSetXFormation(drivetrain));
		/*	
		driverController.x()
			.whileTrue(new Out(climber));

		driverController.y()
			.whileTrue(new Climb(climber));
		*/
		
		//driverController.x()
		//	.onTrue(new BlastAlgae(algaeBlaster, telescopingArm));

		// driverController.rightBumper()
		// 	.onTrue(new DrivetrainTurnUsingCamera(drivetrain, apriltag_camera));
			
		// driverController.leftStick()
		// 	.onTrue(new DrivetrainSetXFormation(drivetrain));

		// driverController.rightStick()
		// 	.onTrue(new DrivetrainTurnUsingCamera(drivetrain, object_detection_camera));
			

		// co-driver controls
		//coDriverController.rightStick()
		//	.whileTrue(new ManuallyAdjustTelescopingArm(telescopingArm, coDriverController));

		// speed factor (smaller is faster)
		
		/*
		coDriverController.leftStick()
			.whileTrue(new ManuallyAdjustPivotArm(pivotArm, coDriverController));

		coDriverController.a()
			.onTrue(pivotArm.setGoalDegreesCommand(Rotation2d.fromRadians(.4).getDegrees()));
		*/

		// coDriverController.b()
		// 	.whileTrue(new Intake(coralator));

		/*
		coDriverController.b()
		.onTrue(pivotArm.setGoalDegreesCommand(Rotation2d.fromRadians(1).getDegrees()));


		coDriverController.y()
			.whileTrue(new Eject(coralator));

		coDriverController.x()
			.onTrue(pivotArm.setGoalDegreesCommand(pivotArm.getPosition()));

		coDriverController.leftBumper().whileTrue(new IntakeAlgae(algaeBlaster));
		coDriverController.rightBumper().whileTrue(new BlastAlgae(algaeBlaster));

		coDriverController.povUp().onTrue(pivotArm.setSpeedFactorCommand(50));
		coDriverController.povDown().onTrue(pivotArm.setSpeedFactorCommand(150));
		*/

		// Characterization commands on third joystick POV
		
		/*
		characterizationController.povUp()
			.whileTrue(pivotArm.quasistaticForward());

		characterizationController.povDown()
			.whileTrue(pivotArm.quasistaticBackward());

		characterizationController.povRight()
			.whileTrue(pivotArm.dynamicForward());

		characterizationController.povLeft()
			.whileTrue(pivotArm.dynamicBackward());
		*/
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		autonOption = autonOptionChooser.getSelected();
		System.out.println("Auton option: " + autonOption);
		
		switch (autonOption) {
			case AUTON_SIMPLE_DRIVE:
				return new RunCommand(
					() -> drivetrain.drive(-0.2, 0, 0, false, false),
					drivetrain)
					.withTimeout(2);
			
			case AUTON_DRIVE_PLUS:
				return new RunCommand(
					() -> drivetrain.drive(-0.3, 0, 0, false, false),
					drivetrain)
					.withTimeout(4);
			
			case AUTON_DO_NOTHING:
			default:
				return null;
		} 
	}

	public TrajectoryConfig createFastTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.HIGH_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}

	public TrajectoryConfig createTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.MAX_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}

	public TrajectoryConfig createSlowTrajectoryConfig() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
			AutoConstants.REDUCED_SPEED_METERS_PER_SECOND,
			AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DrivetrainConstants.DRIVE_KINEMATICS);

		return config;
	}


	public TrajectoryConfig createReverseTrajectoryConfig() {

		TrajectoryConfig config = createTrajectoryConfig();

		config.setReversed(true); // in reverse!

		return config;
	}

	public TrajectoryConfig createFastReverseTrajectoryConfig() {

		TrajectoryConfig config = createFastTrajectoryConfig();

		config.setReversed(true); // in reverse!

		return config;
	}
	
	public Field2d getField()
	{
		return field;
	}

	public HMAccelerometer getAccelerometer()
	{
		return accelerometer;
	}

	/*
	public ICamera getObjectDetectionCamera()
	{
		return object_detection_camera;
	}
	*/

	/*
	public ICamera getAprilTagCamera()
	{
		return apriltag_camera;
	}
	*/

	public SwerveDrivetrain getDrivetrain()
	{
		return drivetrain;
	}

	public SendableChooser<String> getAutonChooser()
	{
		return autonChooser;
	}

	public SendableChooser<String> getAutonOptionChooser()
	{
		return autonOptionChooser;
	}
}
