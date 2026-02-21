// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/*import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;*/
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import frc.robot.Constants.SwerveModuleConstants;

/**
 * The {@code SwerveModule} class contains fields and methods pertaining to the function of a swerve module.
 */
public class SwerveModule {
	private final SparkMax m_drivingSparkMax;
	private final SparkMax m_turningSparkMax;
	private final SparkMaxConfig drivingConfig;
	private final SparkMaxConfig turningConfig;

	private final RelativeEncoder m_drivingEncoder;
	private final RelativeEncoder m_turningEncoder;
	private final SparkAbsoluteEncoder m_turningAbsoluteEncoder;

	private final SparkClosedLoopController m_drivingClosedLoopController;
	private final SparkClosedLoopController m_turningClosedLoopController;

	private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

	/**
	 * Constructs a SwerveModule and configures the driving and turning motor,
	 * encoder, and PID controller.
	 */
	public SwerveModule(int drivingCANId, int turningCANId) {
		double turningFactor = 2 * Math.PI;
		drivingConfig = new SparkMaxConfig();
		turningConfig = new SparkMaxConfig();

		m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
		m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

		// Setup encoders and PID controllers for the driving and turning SPARKS MAX.
		m_drivingEncoder = m_drivingSparkMax.getEncoder();
		m_turningEncoder = m_turningSparkMax.getEncoder();
		m_turningAbsoluteEncoder = m_turningSparkMax.getAbsoluteEncoder();

		m_drivingClosedLoopController = m_drivingSparkMax.getClosedLoopController();
		m_turningClosedLoopController = m_turningSparkMax.getClosedLoopController();

		drivingConfig
			.inverted(false)
			.idleMode(SwerveModuleConstants.DRIVING_MOTOR_IDLE_MODE)
			.smartCurrentLimit(SwerveModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT_AMPS);
		drivingConfig.encoder
			.positionConversionFactor(SwerveModuleConstants.DRIVING_ENCODER_POSITION_FACTOR_METERS_PER_ROTATION)
			.velocityConversionFactor(SwerveModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR_METERS_PER_SECOND_PER_RPM);
		drivingConfig.closedLoop
			.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
			.pid(SwerveModuleConstants.DRIVING_P, SwerveModuleConstants.DRIVING_I, SwerveModuleConstants.DRIVING_D)
			.velocityFF(SwerveModuleConstants.DRIVING_FF)
            .outputRange(SwerveModuleConstants.DRIVING_MIN_OUTPUT_NORMALIZED, SwerveModuleConstants.DRIVING_MAX_OUTPUT_NORMALIZED);
		m_drivingSparkMax.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		turningConfig
			.inverted(true)
            .idleMode(SwerveModuleConstants.TURNING_MOTOR_IDLE_MODE)
            .smartCurrentLimit(SwerveModuleConstants.TURNING_MOTOR_CURRENT_LIMIT_AMPS);
        turningConfig.absoluteEncoder
            .positionConversionFactor(SwerveModuleConstants.TURNING_ENCODER_POSITION_FACTOR_RADIANS_PER_ROTATION) // radians
            .velocityConversionFactor(SwerveModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR_RADIANS_PER_SECOND_PER_RPM); // radians per second
        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            // These are example gains you may need to them for your own robot!
            .pid(SwerveModuleConstants.TURNING_P, SwerveModuleConstants.TURNING_I, SwerveModuleConstants.TURNING_D)
            .outputRange(SwerveModuleConstants.TURNING_MIN_OUTPUT_NORMALIZED, SwerveModuleConstants.TURNING_MAX_OUTPUT_NORMALIZED)
			.velocityFF(SwerveModuleConstants.TURNING_FF)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, turningFactor);
		m_turningSparkMax.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		m_desiredState.angle = new Rotation2d(m_turningAbsoluteEncoder.getPosition());
		m_drivingEncoder.setPosition(0);
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		return new SwerveModuleState(m_drivingEncoder.getVelocity(),
			new Rotation2d(m_turningAbsoluteEncoder.getPosition()));
	}

	/**
	 * Returns the current position of the module.
	 *
	 * @return The current position of the module.
	 */
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(
			m_drivingEncoder.getPosition(),
			new Rotation2d(m_turningAbsoluteEncoder.getPosition()));
	}

	/**
	 * Sets the desired state for the module.
	 *
	 * @param desiredState Desired state with speed and angle.
	 */
	public void setDesiredState(SwerveModuleState desiredState) {
		// Apply chassis angular offset to the desired state.
		SwerveModuleState correctedDesiredState = new SwerveModuleState();
		correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
		correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(0));

		// Optimize the reference state to avoid spinning further than 90 degrees.
		correctedDesiredState.optimize(new Rotation2d(m_turningAbsoluteEncoder.getPosition()));

		if (Math.abs(correctedDesiredState.speedMetersPerSecond) < 0.001 // less than 1 mm per sec
			&& Math.abs(correctedDesiredState.angle.getRadians() - m_turningAbsoluteEncoder.getPosition()) < Rotation2d.fromDegrees(1).getRadians()) // less than 1 degree
		{
			m_drivingSparkMax.set(0); // no point in doing anything
			m_turningSparkMax.set(0);
		}
		else
		{
			// Command driving and turning SPARKS MAX towards their respective setpoints.
			m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
			m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
		}

		m_desiredState = desiredState;
	}

	public RelativeEncoder getDrivingEncoder()
	{
		return m_drivingEncoder;
	}

	public RelativeEncoder getTurningEncoder()
	{
		return m_turningEncoder;
	}

	public SparkAbsoluteEncoder getTurningAbsoluteEncoder()
	{
		return m_turningAbsoluteEncoder;
	}

	public SwerveModuleState getDesiredState()
	{
		return m_desiredState;
	}
	/** Zeroes the SwerveModule drive encoders. */
	public void resetEncoders() {
		m_drivingEncoder.setPosition(0);
	}
}