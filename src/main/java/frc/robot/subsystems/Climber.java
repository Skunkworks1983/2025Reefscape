// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  TalonFX climbMotor;
  enum direction {
    UP,
    STATIONARY,
    DOWN
  }

   public Climber() {
    //instantiates climb motor
    climbMotor = new TalonFX(0);

    //instantiates enum
    direction up = direction.UP;
    direction stationary = direction.STATIONARY;
    direction down = direction.DOWN;
   }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void move(direction myVar) {
    
    switch(myVar) 
    {
      //when i ask it to go up, it go up
      case UP:
        System.out.println("TBD");
        climbMotor.set(.01);
        break;

      //when i ask it to stay, it stay
      case STATIONARY:
        System.out.println("TBD");
        break;

      //when i ask it to go down, it go down
      case DOWN:
        System.out.println("TBD");
        climbMotor.set(-.01);
        break;
    }

  }
}
