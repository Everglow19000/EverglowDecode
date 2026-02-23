package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.everglow_library.Subsystem;
import org.firstinspires.ftc.teamcode.everglow_library.Utils;

public class Shooter implements Subsystem {
    Robot robot;
    private static double[] LUTsDistances = new double[] {
            15,
            45,
            75,
            110,
            135
    };

    // --------------------
    // | Servo Parameters |
    // --------------------
    public static double minServoPosition = 0.51;
    public static double maxServoPosition = 0.71;
    public static double minServoAngle = 3.0;
    public static double maxServoAngle = 42.5;
    private static Servo.Direction servoDirection = Servo.Direction.REVERSE;
    private static InterpLUT servoAnglesLUT = Utils.interpLUTFromArrays( //TODO: FILL ME WITH MEASURED VALUES
            LUTsDistances,
            new double[]{
                    11.5,
                    30,
                    38,
                    38,
                    45
            }
    );


    // -----------------------
    // | Flywheel Parameters |
    // -----------------------
    private boolean flywheelShouldSpin = false;
    private static boolean isFlywheel1Inverted = true;
    private static boolean isFlywheel2Inverted = false;
    private static InterpLUT flywheelSpeedsLUT = Utils.interpLUTFromArrays( //TODO: FILL ME WITH MEASURED VALUES
            LUTsDistances,
            new double[]{
                    1220,
                    1280,
                    1420,
                    1540,
                    1680
            }
    );

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
        private double givenDistanceFromGoal;

        private StartUpShooterAction(double distanceFromGoal) {
            this.givenDistanceFromGoal = distanceFromGoal;
            desiredFlywheelSpeed = getFlywheelTicksPerSecondForDistanceFromGoal(distanceFromGoal);
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            givenDistanceFromGoal = robot.calculateDistanceFromGoal();
            desiredFlywheelSpeed = getFlywheelTicksPerSecondForDistanceFromGoal(givenDistanceFromGoal);
            flywheelShouldSpin = true;
            robot.update();

            return true;
        }
    }

    public class AimHoodAction implements Action {
        private double targetPosition;

        private AimHoodAction(double pos) {
            targetPosition = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            setHoodServoAngle(getServoAngleForDistanceFromGoal(robot.calculateDistanceFromGoal()));

            return true;
        }
    }

    public class WaitUntilShooterSpinupAction implements Action {
        private WaitUntilShooterSpinupAction() {
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return !isFlywheelFinishedSpinning();
        }
    }


    MotorEx flywheelMotor1;
    MotorEx flywheelMotor2;
    Servo hoodServo;
    public PIDFController flywheelPIDF = new PIDFController(0.025, 0.2, 0, 0.0001);
    public double desiredFlywheelSpeed = 0; // [ticks/s]
    private double targetServoPosition = 0;


    public Shooter(HardwareMap hardwareMap, Robot robot) {
        this.robot = robot;
        flywheelMotor1 = new MotorEx(hardwareMap, "flywheelMotor", Motor.GoBILDA.BARE);

        flywheelMotor1.setInverted(isFlywheel1Inverted);
        flywheelMotor1.setRunMode(Motor.RunMode.RawPower);

        flywheelMotor2 = new MotorEx(hardwareMap, "flywheelMotor2", Motor.GoBILDA.BARE);

        flywheelMotor2.setInverted(isFlywheel2Inverted);
        flywheelMotor2.setRunMode(Motor.RunMode.RawPower);

        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        hoodServo.setDirection(servoDirection);
    }

    public double getFlywheelTicksPerSecondForDistanceFromGoal(double distance) {
        return flywheelSpeedsLUT.get(clampDistance(distance));
    }
    public double getServoAngleForDistanceFromGoal(double distance) {
        return servoAnglesLUT.get(clampDistance(distance));
    }


    public void setFlywheelMotorSpeed(double ticksPerSecond) {
        if (ticksPerSecond == 0) {
            stopMotor();
            return;
        }
        flywheelShouldSpin = true;
        desiredFlywheelSpeed = ticksPerSecond;
    }

    public double hoodDegreesToServoPosition(double hoodDegrees) {
        return ((hoodDegrees-minServoAngle)/(maxServoAngle-minServoAngle));
    }
    public double servoPositionToHoodDegrees(double position) {
        return minServoAngle + (position*(maxServoAngle-minServoAngle));
    }

    public double getFlywheelCurrentRPM() {
        return (flywheelMotor1.getCorrectedVelocity()/flywheelMotor1.getCPR())*60.0;
    }

    public double getFlywheel1Power() {
        return flywheelMotor1.get();
    }
    public double getFlywheel2Power() {
        return flywheelMotor2.get();
    }

    public double getFlywheelMotorCurrentTicksPerSecond() {
        return flywheelMotor1.getCorrectedVelocity();
    }

    private double clampDistance(double distance) {
        return Math.max(
                LUTsDistances[0] + 0.1,
                Math.min(
                        LUTsDistances[LUTsDistances.length - 1] - 0.1,
                        distance
                )
        );
    }

    public void setHoodServoAngle(double angle) {
        setHoodServoPosition(hoodDegreesToServoPosition(angle));
    }

    public void setHoodServoPosition(double pos) {
        pos = minServoPosition + ((maxServoPosition-minServoPosition)*pos);
        targetServoPosition = pos;
        hoodServo.setPosition(pos);
    }

    public double getHoodServoPosition() {
        return (targetServoPosition - minServoPosition)/(maxServoPosition-minServoPosition);
    }

    public void stopMotor() {
        flywheelMotor1.stopMotor();
        flywheelMotor2.stopMotor();
        flywheelShouldSpin = false;
        desiredFlywheelSpeed = 0;
    }

    public boolean isFlywheelFinishedSpinning() {
        return Math.abs(flywheelMotor1.getCorrectedVelocity() - desiredFlywheelSpeed) <= 20;
    }

    public StopShooterSpinAction getStopShooterSpinAction() {
        return new StopShooterSpinAction();
    }

    public StartUpShooterAction getStartUpShooterAction(double distanceFromGoal) {
        return new StartUpShooterAction(distanceFromGoal);
    }
    public WaitUntilShooterSpinupAction getWaitUntilShooterSpinupAction() {
        return new WaitUntilShooterSpinupAction();
    }

    public AimHoodAction getAimHoodAction(double angle) {
        return new AimHoodAction(hoodDegreesToServoPosition(angle));
    }

    @Override
    public void update(int iterationCount) {
        if (flywheelShouldSpin) {
            double calculatedPower = flywheelPIDF.calculate(
                    getFlywheelMotorCurrentTicksPerSecond(),
                    desiredFlywheelSpeed
            );
            flywheelMotor1.set(calculatedPower);
            flywheelMotor2.set(calculatedPower);
        }
    }

    @Override
    public String status() {
        return "";
    }
}
