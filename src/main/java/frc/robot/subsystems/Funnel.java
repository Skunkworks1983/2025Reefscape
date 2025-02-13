// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {

  private SparkMax pivotMotor;

  public Funnel(int pivotMotorId) {
    this.pivotMotor = new SparkMax(pivotMotorId, MotorType.kBrushless);
  }

  public void maintainPosition(Rotation2d position) {
    
  }

  @Override
  public void periodic() {
  }
}
