package frc.robot.subsystems.Elevator;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final TalonFX m_leaderMotor = new TalonFX(ElevatorConstants.kLeaderMotorID);
  private final TalonFX m_followerMotor = new TalonFX(ElevatorConstants.kFollowerMotorID);
  private Distance goalHeight = Inches.of(0);
  private final MotionMagicExpoTorqueCurrentFOC m_motionMagicReq = new MotionMagicExpoTorqueCurrentFOC(0);
  private final VoltageOut m_voltReq = new VoltageOut(0);
  private final SysIdRoutine routine = new SysIdRoutine(
    new Config(null, null, null), 
    new Mechanism(null, null, null)
  );


  public Elevator () {
    m_leaderMotor.getConfigurator().apply(ElevatorConstants.kElevatorConfigs);
    m_followerMotor.getConfigurator().apply(ElevatorConstants.kFolllowerConfigs);
    m_followerMotor.setControl(new Follower(ElevatorConstants.kLeaderMotorID, true));
  }

  public Command zeroEncoder(){
    return this.startEnd(
      ()->m_leaderMotor.setControl(new VoltageOut(Volts.of(-2))),
      ()->m_leaderMotor.setControl(new NeutralOut())
    ).finallyDo(
      (interrupt)->{ if (!interrupt){ m_leaderMotor.setPosition(Degrees.of(0)); } }
    ).until(
      ()->m_followerMotor.getStatorCurrent().getValue().gt(Amps.of(20))
    );
  }

  public Command setHeightCommand(Distance Height){
    goalHeight = Inches.of(MathUtil.clamp(Height.in(Inches), 0, ElevatorConstants.kElevatorMaxHeight.in(Inches)));
    return this.startEnd(()->m_leaderMotor.setControl(m_motionMagicReq.withPosition(goalHeight.timesConversionFactor(ElevatorConstants.kConversion.reciprocal()))), null)
    .until(this::isAtGoal);
  }
  
  public boolean isAtGoal(){
    //return MathUtil.isNear(goalHeight.in(Inches), m_leaderMotor.getPosition().getValueAsDouble() * ElevatorConstants.kElevatorRatio * ElevatorConstants.kSprocketPitchCircumference.in(Inches),0.1);
    return m_leaderMotor.getPosition().getValue().timesConversionFactor(ElevatorConstants.kConversion).isNear(goalHeight, Inches.of(0.1));
  }

  private void setVolt(Voltage volts){
    m_leaderMotor.setControl(m_voltReq.withOutput(volts));
  }

  // unused periodic method
  @Override
  public void periodic() {}
}

