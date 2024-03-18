// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter() {}
  private final PWMSparkMax shooterMotor = new PWMSparkMax(1);
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void shooterOn(){
    shooterMotor.set(1);

  }
  public void shooterOff(){
    shooterMotor.set(0);
  }
    public void shooterAmp(){
    shooterMotor.set(.205);
  } 
  
}
