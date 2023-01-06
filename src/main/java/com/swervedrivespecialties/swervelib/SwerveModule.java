package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.kinematics.SwerveModulePosition;

public interface SwerveModule {
    TalonFX getDriveMotor();

    TalonFX getSteerMotor();

    double getDriveVelocity();

    double getDistanceMeters();

    double getSteerAngle();

    SwerveModulePosition getSwerveModulePosition();

    void set(double driveVoltage, double steerAngle);

    void stop();
}
