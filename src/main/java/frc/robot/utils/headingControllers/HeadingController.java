// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.headingControllers;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HeadingController extends SubsystemBase {

  private PIDController controller;
  private Consumer<Double> setHeading;
  private Supplier<Double> getHeading;

  public HeadingController(Consumer<Double> setHeading, Supplier<Double> getHeading) {
    this.controller = new PIDController(0, 0, 0);
    this.setHeading = setHeading;
    this.getHeading = getHeading;
  }

  @Override
  public void periodic() {
    double speed = controller.calculate(getHeading.get());
    setHeading.accept(speed);
  }

  public void updateDesiredHeading(double newHeading) {
    controller.setSetpoint(newHeading);
  }
}
