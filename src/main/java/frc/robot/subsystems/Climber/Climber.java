// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private final TalonFX m_winchMotor = new TalonFX(ClimberConstants.kClimberMotorID);
  private final Servo m_servo = new Servo(ClimberConstants.kLatchServoID);
  private Angle m_goalAngle = Degrees.of(0);

  public Climber() {
    m_winchMotor.getConfigurator().apply(
      ClimberConstants.kWinchConfigs
    );
    m_winchMotor.setPosition(0);
  }

  public Command setWinchPosition(Angle angle){
    
    return this.runOnce(()->{
      m_winchMotor.setControl(new PositionVoltage(angle));
      m_goalAngle = angle;
    }).andThen(Commands.idle(this)).until(this::isAtGoal);
  }

  public boolean isAtGoal(){
    return m_winchMotor.getPosition().getValue().isNear(m_goalAngle, Rotations.of(1));
  }

  public Command setServoLocked(boolean locked){
    return this.runOnce(()->{
      m_servo.set(locked?1:0);
    });
  }

  public Angle winchAngle(){
    return Rotations.of(m_winchMotor.getPosition().getValueAsDouble()/ClimberConstants.kUpAngle.in(Rotations));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("winch position", m_winchMotor.getPosition().getValue().in(Rotation));
  }
}
