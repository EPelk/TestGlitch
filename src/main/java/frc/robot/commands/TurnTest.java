/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class TurnTest extends Command {
    private double startRotation;
    private double startDist;
    private double finalHeading = 0;

//End of things that will be moved later

    public TurnTest() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.drivetrain.setPower(0,0);
        finalHeading = 0;
        Timer.delay(0.2);
        startRotation = Robot.drivetrain.getContinuousHeading();
        startDist = (Robot.drivetrain.getLeftDistance() + Robot.drivetrain.getRightDistance()) / 2;

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if(Math.abs(Robot.drivetrain.getContinuousHeading() - startRotation) < 90) {
            Robot.drivetrain.setPower(0.3, -0.3);
            System.out.println("Moving . . . ");
        }
        else {
          Robot.drivetrain.setPower(0,0);
          
          if (finalHeading == 0){
            finalHeading = Robot.drivetrain.getContinuousHeading();
          }
          SmartDashboard.putNumber("final heading", finalHeading);
        }

    }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
