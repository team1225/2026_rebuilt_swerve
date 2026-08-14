// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants.DrivetrainConstants;
import frc.utils.SwerveUtils;
import frc.robot.Ports;

/**
 * The {@code SwerveDrivetrain} class contains fields and methods pertaining to the function of the drivetrain.
 */
public class SwerveDrivetrain extends SubsystemBase {

	// convention: screw head pointing left/port side

	// calibration: manually move wheels so it's facing straight then record the number below, deploy code then enable :)

	public static final int GYRO_ORIENTATION = 1;

	public static final double FIELD_LENGTH_INCHES = 54*12+1; // 54ft 1in
	public static final double FIELD_WIDTH_INCHES = 26*12+7; // 26ft 7in
	

	// Create SwerveModules
	private final SwerveModule m_frontLeft = new SwerveModule(
		Ports.CAN.FRONT_LEFT_DRIVING,
		Ports.CAN.FRONT_LEFT_TURNING);

	private final SwerveModule m_frontRight = new SwerveModule(
		Ports.CAN.FRONT_RIGHT_DRIVING,
		Ports.CAN.FRONT_RIGHT_TURNING);

	private final SwerveModule m_rearLeft = new SwerveModule(
		Ports.CAN.REAR_LEFT_DRIVING,
		Ports.CAN.REAR_LEFT_TURNING);

	private final SwerveModule m_rearRight = new SwerveModule(
		Ports.CAN.REAR_RIGHT_DRIVING,
		Ports.CAN.REAR_RIGHT_TURNING);

	// The gyro sensor
	private final Pigeon2 m_gyro = new Pigeon2(Ports.CAN.pigeon2); 

	// Slew rate filter variables for controlling lateral acceleration
	private double m_currentRotation = 0.0;
	private double m_currentTranslationDir = 0.0;
	private double m_currentTranslationMag = 0.0;

	private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DrivetrainConstants.MAGNITUDE_SLEW_RATE);
	private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DrivetrainConstants.ROTATIONAL_SLEW_RATE);
	private double m_prevTime = WPIUtilJNI.now() * 1e-6;

	// Odometry class for tracking robot pose
	SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
		DrivetrainConstants.DRIVE_KINEMATICS,
		Rotation2d.fromDegrees(GYRO_ORIENTATION * m_gyro.getYaw().getValueAsDouble()),
		new SwerveModulePosition[] {
			m_frontLeft.getPosition(),
			m_frontRight.getPosition(),
			m_rearLeft.getPosition(),
			m_rearRight.getPosition()
		},
		new Pose2d()
		);


	// other variables

	public double MaxSpeedMultiplier = 1.0;


	/** Creates a new Drivetrain. */
	public SwerveDrivetrain() {

		m_frontLeft.resetEncoders(); // resets relative encoders
		m_frontRight.resetEncoders();
		m_rearLeft.resetEncoders();
		m_rearRight.resetEncoders();

		zeroHeading(); // resets gyro

		// sets initial pose arbitrarily
		// Note: the field coordinate system (or global coordinate system) is an absolute coordinate system where a point on the field is designated as the origin.
		// Positive theta is in the counter-clockwise direction, and the positive x-axis points away from your alliance’s driver station wall,
		// and the positive y-axis is perpendicular and to the left of the positive x-axis.
		Translation2d initialTranslation = new Translation2d(Units.inchesToMeters(FIELD_LENGTH_INCHES/2),Units.inchesToMeters(FIELD_WIDTH_INCHES/2)); // mid field
		Rotation2d initialRotation = new Rotation2d(); 
		Pose2d initialPose = new Pose2d(initialTranslation,initialRotation);
		resetOdometry(initialPose);


	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block
		m_poseEstimator.update(
			Rotation2d.fromDegrees(GYRO_ORIENTATION * m_gyro.getYaw().getValueAsDouble()),
			new SwerveModulePosition[] {
				m_frontLeft.getPosition(),
				m_frontRight.getPosition(),
				m_rearLeft.getPosition(),
				m_rearRight.getPosition()
			});

			LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
			LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
			
			boolean doRejectUpdate = false;
			// if our angular velocity is greater than 360 degrees per second, ignore vision updates
			if(Math.abs(m_currentRotation) > 360)
			{
				doRejectUpdate = true;
			}
			if(mt2.tagCount == 0)
			{
				doRejectUpdate = true;
			}
			if(!doRejectUpdate)
			{
				m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
				m_poseEstimator.addVisionMeasurement(
					mt2.pose,
					mt2.timestampSeconds);
			}




	}


	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		m_poseEstimator.resetPosition(
			Rotation2d.fromDegrees(GYRO_ORIENTATION * m_gyro.getYaw().getValueAsDouble()),
			new SwerveModulePosition[] {
				m_frontLeft.getPosition(),
				m_frontRight.getPosition(),
				m_rearLeft.getPosition(),
				m_rearRight.getPosition()
			},
			pose);
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 * @param rateLimit     Whether to enable rate limiting for smoother control.
	 */
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
		
		double xSpeedCommanded;
		double ySpeedCommanded;

		if (rateLimit) {
			// Convert XY to polar for rate limiting
			double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
			double inputTranslationMag = Math.sqrt(Math.pow(xSpeed/2, 2) + Math.pow(ySpeed/2, 2));

			// Calculate the direction slew rate based on an estimate of the lateral acceleration
			double directionSlewRate;

			if (m_currentTranslationMag != 0.0) {
				directionSlewRate = Math.abs(DrivetrainConstants.DIRECTION_SLEW_RATE / m_currentTranslationMag);
			} else {
				directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
			}
			

			double currentTime = WPIUtilJNI.now() * 1e-6;
			double elapsedTime = currentTime - m_prevTime;
			double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);

			if (angleDif < 0.45*Math.PI) {
				m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
				m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
			}
			else if (angleDif > 0.85*Math.PI) {
				if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
					// keep currentTranslationDir unchanged
					m_currentTranslationMag = m_magLimiter.calculate(0.0);
				}
				else {
					m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
					m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
				}
			}
			else {
				m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
				m_currentTranslationMag = m_magLimiter.calculate(0.0);
			}

			m_prevTime = currentTime;
			
			xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
			ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
			m_currentRotation = m_rotLimiter.calculate(rot);

		} else {
			xSpeedCommanded = xSpeed;
			ySpeedCommanded = ySpeed;
			m_currentRotation = rot;
		}

		// Convert the commanded speeds into the correct units for the drivetrain
		double xSpeedDelivered = xSpeedCommanded * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND * this.MaxSpeedMultiplier;
		double ySpeedDelivered = ySpeedCommanded * DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND * this.MaxSpeedMultiplier;
		double rotDelivered = m_currentRotation * DrivetrainConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND * this.MaxSpeedMultiplier;

		var swerveModuleStates = DrivetrainConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
			fieldRelative
				? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, 
					Rotation2d.fromDegrees(m_poseEstimator.getEstimatedPosition().getRotation().getDegrees()))
				: new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

		SwerveDriveKinematics.desaturateWheelSpeeds(
			swerveModuleStates, DrivetrainConstants.MAX_SPEED_METERS_PER_SECOND * this.MaxSpeedMultiplier);

		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_rearLeft.setDesiredState(swerveModuleStates[2]);
		m_rearRight.setDesiredState(swerveModuleStates[3]);
	}


	/**
	 * Sets the wheels into an X formation to prevent movement.
	 */
	public void setX() {
		m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
	}
	

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		m_gyro.reset();
		m_gyro.setYaw(0);
	}


	public void stop()
	{
		drive(0, 0, 0, false, false);
	}

}