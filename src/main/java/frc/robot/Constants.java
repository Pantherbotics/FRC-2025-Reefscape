// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class Constants {

    public static class DrivetrainConstants{

        public static final PIDController kXController = new PIDController(1, 0, 0);
        public static final PIDController kYController = new PIDController(1, 0, 0);
        public static final PIDController kHeadingController = new PIDController(1, 0, 0);
    }

    public static class VisionConstants{
        public static final String kLeftCamName = "leftCam";
        public static final String kRightCamName = "rightCam";
        public static final Transform3d kRobotToLeftCamTransform = new Transform3d(Units.inchesToMeters(-15), 0, Units.inchesToMeters(7), new Rotation3d(0, 0, Units.degreesToRadians(180)));
        public static final Transform3d kRobotToRightCamTransform = new Transform3d(Units.inchesToMeters(-15), 0, Units.inchesToMeters(7), new Rotation3d(0, 0, Units.degreesToRadians(180)));

        public static final Transform2d kLeftPositionOffset = new Transform2d(Units.inchesToMeters(20), Units.inchesToMeters(10), Rotation2d.fromDegrees(180));
    };

    public static class ElevatorConstants{
        public static final int kLeaderMotorID = 16;
        public static final int kFollowerMotorID = 17;
        public static final double kElevatorRatio = (16d/48d) * (36d/48d) * (30d/42d);
        public static final Distance kSprocketCircumference = Inches.of(24 * 0.25); // 24 teeth on 1/4" pitch
        public static final Distance kElevatorMaxHeight = Inches.of(31);
        public static final Per<DistanceUnit, AngleUnit> kConversion = kSprocketCircumference.div(Rotations.of(1/kElevatorRatio));
        
        public static final Voltage kZeroVoltage = Volts.of(-0.5);

        public static final TalonFXConfiguration kElevatorConfigs = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                .withKP(66)//450.42)
                .withKI(0)
                .withKD(1.3)//1.943)
                .withKS(0.20929)
                .withKV(0.12342)
                .withKG(0.026837)
                .withKA(0.0016769)
                .withGravityType(GravityTypeValue.Elevator_Static)
            )
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicAcceleration(90)
                .withMotionMagicCruiseVelocity(100)
                .withMotionMagicExpo_kV(0.12342)
                .withMotionMagicExpo_kA(0.0016769)
            )
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(10)
                .withSupplyCurrentLimit(100)
            )
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive)
            )
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(kElevatorMaxHeight.in(Inches)*kSprocketCircumference.in(Inches) * (1/kElevatorRatio))
                .withForwardSoftLimitEnable(false)
                .withReverseSoftLimitThreshold(0)
                .withReverseSoftLimitEnable(false)
            );

        public static final TalonFXConfiguration kFolllowerConfigs = 
            kElevatorConfigs.withMotorOutput(new MotorOutputConfigs()
              .withInverted(InvertedValue.CounterClockwise_Positive)  
            );
                

    }

    


}
