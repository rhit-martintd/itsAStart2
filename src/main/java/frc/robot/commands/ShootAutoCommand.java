// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.IntermediateWheel;

public class ShootAutoCommand extends Command {
  /** Creates a new ShootCommand. */
  private IntermediateWheel interwheelCommand;
  private Shooter shooterCommand;
  private Timer timer = new Timer();
  private int timerDelay;

  public ShootAutoCommand(Shooter shoot, IntermediateWheel interwheel, int timerValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoot);
    addRequirements(interwheel);
    this.shooterCommand = shoot;
    this.interwheelCommand = interwheel;
    this.timerDelay = timerValue;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timer.reset();
    this.timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooterCommand.shooterOn();
    if(this.timer.get()>=.2) {
      this.interwheelCommand.wheelOn();
      }
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

   this.shooterCommand.shooterOff();
   this.interwheelCommand.wheelOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() >= this.timerDelay) {
      this.shooterCommand.shooterOff();
      this.interwheelCommand.wheelOff();
      return true;
    } else {
      return false;
    }
    
    
  }
}
