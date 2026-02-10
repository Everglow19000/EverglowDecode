package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;

public class Turret implements Subsystem  {
    // how many ticks the motor has from it's homing point to its max
    public static int MAX_TICK_RANGE = 1000;
    // what angle can the turret rotate (due to the mechanical stop)
    public static double MAX_ANGLE_RANGE = Math.toRadians(355);
    // at what offset is the 0 position angle is to the robot's heading
    public static double ANGLE_OFFSET_TO_ROBOT = Math.toRadians(45);

    private int encoderPositionOffset = 0;
    public MotorEx turretMotor;
    // what position motor was last time checked. useful for AUTO -> TELE transition.
    public static int lastPosition;
    public static boolean isLastPositionSet = false;

    public Turret(HardwareMap hardwareMap) {
        this.turretMotor = new MotorEx(hardwareMap, "turretMotor", Motor.GoBILDA.RPM_312);

        if (isLastPositionSet) {
            this.encoderPositionOffset = lastPosition;
        }
    }

    @Override
    public void update(int iterationCount) {
        lastPosition = turretMotor.getCurrentPosition();
        isLastPositionSet = true;
    }

    @Override
    public String status() {
        return "";
    }
}
