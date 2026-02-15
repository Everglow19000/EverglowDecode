package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;

public class Turret implements Subsystem  {
    public static double HOME_TURRET_POWER = 0.5;
    public static double MECHANICAL_STOPPER_CURRENT_MILLIAMPS = 750*(HOME_TURRET_POWER*10);
    public class HomeTurretAction implements Action {
        boolean hasStarted = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!hasStarted) {
                hasStarted = true;
                turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turretMotor.setPower(-HOME_TURRET_POWER);
            }

            if (turretMotor.getCurrent(CurrentUnit.MILLIAMPS) > MECHANICAL_STOPPER_CURRENT_MILLIAMPS) {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(MOTOR_POWER);
                return false;
            }
            return true;
        }
    }
    // how many ticks the motor has from it's homing point to its max
    public static int MAX_TICK_RANGE = 1730;
    // what angle can the turret rotate (due to the mechanical stop)
    public static Rotation2d MAX_ANGLE_RANGE = Rotation2d.exp(Math.toRadians(350));
    // at what offset is the 0 position angle is to the robot's heading
    public static Rotation2d ANGLE_OFFSET_TO_ROBOT = Rotation2d.exp(Math.toRadians(50));
    public static DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static double RADIANS_TO_TICKS_CONSTANT = (MAX_TICK_RANGE/rotation2dToTwoPIRange(MAX_ANGLE_RANGE));
    public static double MOTOR_POWER = 0.8;
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
        double otherSideHomeAngle = rotation2dToTwoPIRange(ANGLE_OFFSET_TO_ROBOT) - (Math.PI*2 - rotation2dToTwoPIRange(MAX_ANGLE_RANGE));

        double vectorAngle = rotation2dToTwoPIRange(rotation);

        if (vectorAngle < homeAngle && vectorAngle > otherSideHomeAngle) {
            return Math.abs(vectorAngle - homeAngle) < Math.abs(vectorAngle - otherSideHomeAngle) ? ANGLE_OFFSET_TO_ROBOT : Rotation2d.exp(otherSideHomeAngle);
        }
        else {
            return rotation;
        }
    }
    // gets a rotation in local space (so 0 is the robot's heading) and converts it to motor ticks.
    public int wantedAngleToTicks(Rotation2d rotation) {
        if (rotation != null) {
            rotation = validateRotation(rotation);
            double vectorAngle = rotation2dToTwoPIRange(Rotation2d.exp(rotation2dToTwoPIRange(rotation) - rotation2dToTwoPIRange(ANGLE_OFFSET_TO_ROBOT)));

            return (int)(vectorAngle*RADIANS_TO_TICKS_CONSTANT);
        }
        return -9999999;
    }

    public static Rotation2d globalRotationToLocal(Rotation2d wantedRotation, Rotation2d robotHeading) {
        return wantedRotation.times(robotHeading.inverse());
    }

    public HomeTurretAction getHomeTurretAction() {
        return new HomeTurretAction();
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
