// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {}
  private final PWMVictorSPX frontMotor = new PWMVictorSPX(4);
  private final PWMVictorSPX greenMotor = new PWMVictorSPX(2);
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void intakeOn() {
    frontMotor.set(.5);
    greenMotor.set(.5);
  }

  public void intakeOff(){
    frontMotor.set(0);
    greenMotor.set(0);
  }

  public void intakeReverse(){
    frontMotor.set(-.5);
    greenMotor.set(-.5);
  }
}
