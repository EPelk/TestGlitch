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
import frc.robot.NoTargetException;
import frc.robot.subsystems.Target;

public class SmoothTurn extends Command {

//Variables and constants that will be moved to another class later

    private final double SQUINT_BOUND = 3;
    private Target target;
    private double startRotation;
    private double startDist;
    private double finalHeading = 0;

//End of things that will be moved later

    public SmoothTurn() {
        // Use requires() here to declare subsystem dependencies
        requires(Robot.vision);
        requires(Robot.drivetrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.drivetrain.setPower(0,0);
        finalHeading = 0;
        Timer.delay(0.2);
        try {
            target = Robot.vision.getBestTarget();
        } catch(NoTargetException e) {
            end();
        }
        startRotation = Robot.drivetrain.getContinuousHeading();
        startDist = (Robot.drivetrain.getLeftDistance() + Robot.drivetrain.getRightDistance()) / 2;

    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        try {
            target = Robot.vision.getBestTarget();
            startRotation = Robot.drivetrain.getContinuousHeading();
        } catch(NoTargetException e) {
           end();
        }
        if (target == null){
            end();
        }
        if(Math.abs(Robot.drivetrain.getContinuousHeading() - startRotation) < Math.abs(target.squint())) {
            Robot.drivetrain.setPower(0.3 * Math.signum(target.squint()), 0.3 * -Math.signum(target.squint()));
            System.out.println("Moving . . . ");
            SmartDashboard.putString("vision turn status", "VisionTurn turning");
        }
        else {
            if(finalHeading == 0) {
                finalHeading = Robot.drivetrain.getContinuousHeading();
            }
            SmartDashboard.putNumber("final heading", finalHeading);
            Robot.drivetrain.setPower(0.3,0.3);
            SmartDashboard.putString("vision turn status", "VisionTurn Straight");
        }

    }



    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        
        return ((Robot.drivetrain.getLeftDistance() + Robot.drivetrain.getRightDistance()) / 2) >= (target.standoff() - 12);
        
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.drivetrain.setPower(0,0);
        SmartDashboard.putString("vision turn status", "VisionTurn Done");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
