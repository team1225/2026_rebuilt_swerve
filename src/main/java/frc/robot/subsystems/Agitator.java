// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Constants.AgitatorConstants;

public class Agitator extends SubsystemBase {
  
  private final SparkMax motor;
	private final SparkMaxConfig motorConfig;

  public Agitator() {
    motor = new SparkMax(Ports.CAN.AGITATOR, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();    
    motorConfig
			.inverted(false)
      .idleMode(AgitatorConstants.MOTOR_IDLE_MODE)
      .smartCurrentLimit(AgitatorConstants.CURRENT_LIMIT_AMPS);
      motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {}

  public void run() {
    motor.set(AgitatorConstants.AGITATOR_SPEED);
  }

  public void reverse() {
    motor.set(-0.75 * AgitatorConstants.AGITATOR_SPEED);
  }

  public void stop() {
    motor.set(0);
  }
}
