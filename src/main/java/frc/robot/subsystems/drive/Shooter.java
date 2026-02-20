// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.DriveConstants;

public class Shooter extends SubsystemBase {

  // Create shootingModules
  private final Intake_module mIntake_module = new Intake_module(DriveConstants.intakeCanID);
  private final Intake_module mIntake_module2 = new Intake_module(DriveConstants.assistCanID);
  private final Intake_module mIntake_module3 = new Intake_module(DriveConstants.shooterCanID);
  private final Intake_module mIntake_module4 = new Intake_module(DriveConstants.elevatorCanID);

  @Override
  public void periodic() {

  }
  //Commands for main (CANId 15)
  public Command intake_for() {
    return runOnce(
        () -> {mIntake_module.forward();});
  }

  public Command intake_rev() {
    return runOnce(
        () -> {mIntake_module.reverse();});
  }

  public Command intake_stop() {
    return runOnce(
        () -> {mIntake_module.stop();});
  }
  //Commands for assist (CANId 16)
  public Command intake_for2() {
    return runOnce(
      () -> {mIntake_module2.forward2();});
  }
  public Command intake_rev2() {
    return runOnce(
      () -> {mIntake_module2.reverse2();});
  }
  public Command intake_stop2() {
    return runOnce(
      () -> {mIntake_module2.stop2();});
  }

  //Commands for shooter (CANId 17)
  public Command intake_for3() {
    return runOnce(
      () -> {mIntake_module3.forward3();});
  }
    public Command intake_rev3() {
    return runOnce(
      () -> {mIntake_module3.reverse3();});
  }
    public Command intake_stop3() {
    return runOnce(
      () -> {mIntake_module3.stop3();});
  }

  //Commands for shooter (CANId 18)
  public Command intake_for4() {
    return runOnce(
      () -> {mIntake_module4.forward4();});
  }
    public Command intake_rev4() {
    return runOnce(
      () -> {mIntake_module4.reverse4();});
  }
    public Command intake_stop4() {
    return runOnce(
      () -> {mIntake_module4.stop4();});
}
}
