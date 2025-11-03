package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;

public class Shooter implements Subsystem {
    private static double maxDistanceFromGoal = 3; //TODO: REMOVE THIS SINCE THIS IS A PLACEHOLDER

    // --------------------
    // | Servo Parameters |
    // --------------------
    private static double minServoPosition = 0.35;
    private static double maxServoPosition = 0.62;
    private static Servo.Direction servoDirection = Servo.Direction.REVERSE;


    // -----------------------
    // | Flywheel Parameters |
    // -----------------------
    private static double goalHeight = 0.9843; // meters above the field of the lip of the goal
    private static double shooterHeight = 0.3; // meters above the field of artifact as it leaves shooter TODO: Change according to actual robot measurements
    private static DcMotorSimple.Direction flywheelMotorDirection = DcMotorSimple.Direction.REVERSE;

    public class StopShooterSpinAction implements Action {
        private StopShooterSpinAction() {

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            stopMotor();
            return false;
        }
    }

    public class StartUpShooterAction implements Action {
        private boolean hasStarted = false;
        private double givenDistanceFromGoal;

        private StartUpShooterAction(double distanceFromGoal) {
            this.givenDistanceFromGoal = distanceFromGoal;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!hasStarted) {
                startMotorForDistanceFromGoal(givenDistanceFromGoal);
                aimHoodForDistanceFromGoal(givenDistanceFromGoal);

                hasStarted = true;
            }

            return !isFlywheelFinishedSpinning();
        }
    }


    DcMotorEx flywheelMotor;
    Servo hoodServo;


    public Shooter(HardwareMap hardwareMap) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flyWheelMotor");

        flywheelMotor.setDirection(flywheelMotorDirection);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        hoodServo.setDirection(servoDirection);
        hoodServo.scaleRange(minServoPosition, maxServoPosition);
    }

    /**
     * distance is given in meters, and is the distance from the robot's shooter to the goal.
     * @param distance
     */
    public void startMotorForDistanceFromGoal(double distance) {
        flywheelMotor.setPower(distance/maxDistanceFromGoal); //TODO: REMOVE THIS SINCE THIS IS A PLACEHOLDER
    }

    /**
     * distance is given in meters, and is the distance from the robot's shooter to the goal.
     * @param distance
     */
    public void aimHoodForDistanceFromGoal(double distance) {
        hoodServo.setPosition(distance/maxDistanceFromGoal); //TODO: REMOVE THIS SINCE THIS IS A PLACEHOLDER
    }

    public void stopMotor() {
        flywheelMotor.setPower(0);
    }

    public boolean isFlywheelFinishedSpinning() {
        return true;
    }

    public StopShooterSpinAction getStopShooterSpinAction() {
        return new StopShooterSpinAction();
    }

    public StartUpShooterAction getStartUpShooterAction(double distanceFromGoal) {
        return new StartUpShooterAction(distanceFromGoal);
    }

    @Override
    public void update(int iterationCount) {

    }

    @Override
    public String status() {
        return "";
    }
}
