package com.swervedrivespecialties.swervelib;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public interface SteerController {
    TalonFX getMotor();
    
    double getReferenceAngle();

    void setReferenceAngle(double referenceAngleRadians);

    double getStateAngle();

    void stop();
}
