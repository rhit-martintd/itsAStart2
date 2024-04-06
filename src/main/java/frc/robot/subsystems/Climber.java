// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public Climber() {}
  private final PWMVictorSPX climberMotor = new PWMVictorSPX(0);
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void climberOn(){
    climberMotor.set(.65);
  }
  public void climberOff(){
    climberMotor.set(0);
  }
  public void climberRetract(){
    climberMotor.set(-.65);
  }
}
