/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.LightningMath;
import frc.robot.commands.TankDrive;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  WPI_TalonSRX leftMaster;
  WPI_TalonSRX rightMaster;
  WPI_VictorSPX leftFollow1;
  WPI_VictorSPX leftFollow2;
  WPI_VictorSPX rightFollow1;
  WPI_VictorSPX rightFollow2;

  private AHRS navx;

  public Drivetrain(){
   leftMaster = new WPI_TalonSRX(1);
   rightMaster = new WPI_TalonSRX(4);
   leftFollow1 = new WPI_VictorSPX(2);
   leftFollow2 = new WPI_VictorSPX(3);
   rightFollow1 = new WPI_VictorSPX(5);
   rightFollow2 = new WPI_VictorSPX(6);

   leftMaster.setNeutralMode(NeutralMode.Brake);
   leftFollow1.setNeutralMode(NeutralMode.Brake);
   leftFollow2.setNeutralMode(NeutralMode.Brake);
   rightMaster.setNeutralMode(NeutralMode.Brake);
   rightFollow1.setNeutralMode(NeutralMode.Brake);
   rightFollow2.setNeutralMode(NeutralMode.Brake);

   navx = new AHRS(SPI.Port.kMXP);

   leftFollow1.follow(leftMaster);
   leftFollow2.follow(leftMaster);

   rightFollow1.follow(rightMaster);
   rightFollow2.follow(rightMaster);

    rightMaster.setInverted(!true);
    rightFollow1.setInverted(!false);
    rightFollow2.setInverted(!true);

    leftMaster.setInverted(!false);
    leftFollow1.setInverted(!true);
    leftFollow2.setInverted(!false);

  }

  public void setPower(double l, double r){
    leftMaster.set(ControlMode.PercentOutput, r);
    rightMaster.set(ControlMode.PercentOutput, l);
  }

  public double getContinuousHeading() {
    return navx.getAngle();
}

  public double getLeftDistance(){
    return LightningMath.ticks2inches(leftMaster.getSelectedSensorPosition());
  }

  public double getRightDistance(){
    return LightningMath.ticks2inches(rightMaster.getSelectedSensorPosition());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new TankDrive());
  }
}
