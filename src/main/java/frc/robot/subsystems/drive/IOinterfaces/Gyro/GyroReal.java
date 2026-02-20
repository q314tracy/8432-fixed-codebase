// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive.IOinterfaces.Gyro;

import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.utils.Constants;

/** Add your docs here. */
public class GyroReal implements GyroIO {

    private final Pigeon2 m_gyro;

    public GyroReal() {
        m_gyro = new Pigeon2(30);
    }

    @Override
    public void updatePosition(double pos) {
    }

    @Override
    public void updateRate(double rate) {
    }

    @Override
    public double getAngle() {
        // TUCKER FIZ HIS
        if (Constants.DriveConstants.kGyroReversed) {
            return -m_gyro.getRotation2d().getRadians();
        } else {
            return m_gyro.getRotation2d().getRadians();
        }
    }

    @Override
    public void reset() {
        m_gyro.reset();
    }
}
