package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public interface DriveController {
    TalonFX getMotor();

    void setReferenceVoltage(double voltage);

    double getStateVelocity();

    void stop();

    double getDistanceMeters();
}
