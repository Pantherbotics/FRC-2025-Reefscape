// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.AlgaeIntake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeRollerConstants;
import frc.robot.Constants.RollerConstants;

public class AlgaeRoller extends SubsystemBase {
  private final TalonFXS m_rollerMotor = new TalonFXS(AlgaeRollerConstants.kMotorID);
  private final VoltageOut m_voltReq = new VoltageOut(Volts.of(0));
  private final LaserCan m_laserCan = new LaserCan(AlgaeRollerConstants.kLaserCANID);
  private final Alert m_lasercanAlert = new Alert("Lasercan Bad reading", AlertType.kError);
  /** Creates a new AlgaeRoller. */
  public AlgaeRoller() {
    m_rollerMotor.getConfigurator().apply(AlgaeRollerConstants.kRollerMotorConfigs);
    SmartDashboard.putData(this);
  }

  public Command setVoltage(Voltage voltage){
    return this.runOnce(()->m_rollerMotor.setControl(m_voltReq.withOutput(voltage)))
    .andThen(Commands.idle(this));
  }


  private boolean laserCANReading(){
    LaserCan.Measurement measurement = m_laserCan.getMeasurement();
    // return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT ? measurement.distance_mm < RollerConstants.kThreshold : false;
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      m_lasercanAlert.set(false);
      return measurement.distance_mm < AlgaeRollerConstants.kThreshold;
    } else {
      m_lasercanAlert.set(true);
      return false;
    }
  }

  public boolean hasAlgae() {
    return Utils.isSimulation()?true:laserCANReading();
  }



  @Override
  public void periodic() {
    SmartDashboard.putBoolean("has algae", hasAlgae());
  }
}
