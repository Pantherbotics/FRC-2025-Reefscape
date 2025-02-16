// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Rollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import au.grapplerobotics.LaserCan;

public class Rollers extends SubsystemBase {
  private final LaserCan lc = new LaserCan(RollerConstants.kLaserCANID);

  public Rollers() {}

  public boolean hasTarget(){
    LaserCan.Measurement measurement = lc.getMeasurement();
    // return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT ? measurement.distance_mm < RollerConstants.kThreshold : false;
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return measurement.distance_mm < RollerConstants.kThreshold;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {

  }
}
