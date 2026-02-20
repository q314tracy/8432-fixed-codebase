// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.drive.IOinterfaces.SparkMax.SparkMaxIO;
import frc.robot.subsystems.drive.IOinterfaces.SparkMax.SparkMaxReal;
import frc.robot.subsystems.drive.IOinterfaces.SparkMax.SparkMaxSimulation;
import frc.robot.utils.Configs;

public class Intake_module extends SubsystemBase{
  private final SparkMaxIO M_intake;


  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public Intake_module(int CANId) {

    if (RobotBase.isSimulation()) {
      M_intake = new SparkMaxSimulation(CANId, Configs.intakeModule.intakeConfig);
    } else {
      M_intake = new SparkMaxReal(CANId, Configs.intakeModule.intakeConfig);
    }
  }

  public void forward() {
    M_intake.setVoltage(5);
  }

  /** Used during identification run to check for drive motor inversions. */
  public void reverse() {
    M_intake.setVoltage(-5);
  }

  /** Used during identification run to stop motors. */
  public void stop() {
    M_intake.setVoltage(0);
  }



  //assist intake speed CANID 16
  public void forward2() {
    M_intake.setVoltage(5);
  }
  public void reverse2() {
    M_intake.setVoltage(-5);
  }
  public void stop2() {
    M_intake.setVoltage(0);
  }



  //shooter speed control CANID 17
    public void forward3() {
    M_intake.setVoltage(5);
  }
  public void reverse3() {
    M_intake.setVoltage(-5);
  }
  public void stop3() {
    M_intake.setVoltage(0);
  }


  //elevator CANID 18 speed control
    public void forward4() {
    M_intake.setVoltage(5);
  }
    public void reverse4() {
    M_intake.setVoltage(-5);
  }
    public void stop4() {
    M_intake.setVoltage(0);
  }

  @Override
  public void periodic() {

    //check if simulation, if yes, update simulation data
    if (RobotBase.isSimulation()) {
      M_intake.periodic();
    }
  }
}