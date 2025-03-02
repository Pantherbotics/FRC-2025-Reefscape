// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import frc.robot.subsystems.AlgaeIntake.AlgaePivot;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.EndEffector.Pivot;

/** publishes transformed poses for the articulated model*/
public class Visualizer {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable table = inst.getTable("Pose");
    private final StructArrayTopic<Pose3d> componentTopic = table.getStructArrayTopic("componentPoses", Pose3d.struct);
    private final StructArrayPublisher<Pose3d> componentPub = componentTopic.publish();

    private Pose3d[] poses = new Pose3d[6];

    private final Pivot m_pivot;
    private final Elevator m_elevator;
    private final AlgaePivot m_algaePivot;
    private final Climber m_climber;

    public Visualizer(Pivot pivot, Elevator elevator, AlgaePivot algaePivot, Climber climber){
        this.m_pivot = pivot;
        this.m_algaePivot = algaePivot;
        this.m_elevator = elevator;
        this.m_climber = climber;
        Arrays.fill(poses, new Pose3d());
    }

    public void update(){
        poses[0] = new Pose3d(m_elevator.elevatorPosition().in(Meters) * 0.0871557427477,0,m_elevator.elevatorPosition().in(Meters) * 0.996194698092,Rotation3d.kZero);
        poses[1] = poses[0].plus(
            new Transform3d(
                Inches.of(6.545870), 
                Inches.of(0), 
                Inches.of(13.704344),
                new Rotation3d(Rotations.zero(),m_pivot.pivotAngle().unaryMinus().minus(Degrees.of(90)),Rotations.zero()))
            );
        poses[2] = new Pose3d(
            Units.inchesToMeters(-12.468750),
            0, 
            Units.inchesToMeters(10.150000), 
            new Rotation3d(Rotations.zero(),m_algaePivot.getAngle().minus(Degrees.of(90)),Rotations.zero())
          );
        poses[5] = new Pose3d(
            0,
            Units.inchesToMeters(14), 
            Units.inchesToMeters(11.25), 
            new Rotation3d(m_climber.winchAngle().unaryMinus(),Rotations.zero(),Rotations.zero())
          );

        componentPub.set(poses);
    }
    
}
