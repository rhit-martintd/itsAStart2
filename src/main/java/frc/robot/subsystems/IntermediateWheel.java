// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntermediateWheel extends SubsystemBase {
  /** Creates a new IntermediateWheel. */
  public IntermediateWheel() {}
  private final PWMVictorSPX greenWheel = new PWMVictorSPX(3);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void wheelOn(){
    greenWheel.set(.5);
  }
  public void wheelOff(){
    greenWheel.set(0);
  }
}
