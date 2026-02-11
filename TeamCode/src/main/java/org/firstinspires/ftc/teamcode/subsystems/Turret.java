package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;

public class Turret implements Subsystem  {
    // how many ticks the motor has from it's homing point to its max
    public static int MAX_TICK_RANGE = 4550;
    // what angle can the turret rotate (due to the mechanical stop)
    public static Rotation2d MAX_ANGLE_RANGE = Rotation2d.exp(Math.toRadians(350));
    // at what offset is the 0 position angle is to the robot's heading
    public static Rotation2d ANGLE_OFFSET_TO_ROBOT = Rotation2d.exp(Math.toRadians(45));
    public static DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static double RADIANS_TO_TICKS_CONSTANT = (MAX_TICK_RANGE/rotation2dToTwoPIRange(MAX_ANGLE_RANGE));
    public static double MOTOR_POWER = 1.0;
    private int encoderPositionOffset = 0;
    public DcMotorEx turretMotor;
    // what position motor was last time checked. useful for AUTO -> TELE transition.
    public static int lastPosition;
    public static boolean isLastPositionSet = false;

    public Turret(HardwareMap hardwareMap) {
        this.turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(MOTOR_POWER);
        turretMotor.setDirection(DIRECTION);

        if (isLastPositionSet) {
            this.encoderPositionOffset = lastPosition;
        }
    }


    public static double rotation2dToTwoPIRange(Rotation2d rotation) {
        double angle = rotation.toDouble();
        if (angle < 0) {
            angle += Math.PI*2;
        }
        return angle;
    }

    // makes sure it isn't "on" the mechanical stopper.
    public Rotation2d validateRotation(Rotation2d rotation) {
        double homeAngle = rotation2dToTwoPIRange(ANGLE_OFFSET_TO_ROBOT);
        double maxAllowableAngle = rotation2dToTwoPIRange(MAX_ANGLE_RANGE);

        double vectorAngle = rotation2dToTwoPIRange(Rotation2d.exp(rotation2dToTwoPIRange(rotation) - homeAngle));

        if (vectorAngle < maxAllowableAngle - 0.0001) {
            return rotation;
        }
        else {
            if (vectorAngle > Math.PI + (maxAllowableAngle/2)) {
                return ANGLE_OFFSET_TO_ROBOT;
            }
            else {
                return MAX_ANGLE_RANGE;
            }
        }
    }
    public int wantedAngleToTicks(Rotation2d rotation) {
        rotation = validateRotation(rotation);
        double vectorAngle = rotation2dToTwoPIRange(Rotation2d.exp(rotation2dToTwoPIRange(rotation) - rotation2dToTwoPIRange(ANGLE_OFFSET_TO_ROBOT)));

        return (int)(vectorAngle*RADIANS_TO_TICKS_CONSTANT);
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
