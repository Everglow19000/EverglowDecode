package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="SolversLibTesting", group="Tests")
@Config
public class SolversLibTesting extends LinearOpMode {
    MotorEx testMotor;
    public static double motorPower = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MotorEx flywheelMotor1 = new MotorEx(hardwareMap, "flywheelMotor", Motor.GoBILDA.BARE);

        flywheelMotor1.setInverted(true);
        flywheelMotor1.setRunMode(Motor.RunMode.RawPower);

        MotorEx flywheelMotor2 = new MotorEx(hardwareMap, "flywheelMotor2", Motor.GoBILDA.BARE);

        flywheelMotor2.setInverted(false);
        flywheelMotor2.setRunMode(Motor.RunMode.RawPower);

        waitForStart();

        while (opModeIsActive()) {
            flywheelMotor1.set(motorPower);
            flywheelMotor2.set(motorPower);

            telemetry.addData("motor1 speed", flywheelMotor1.getCorrectedVelocity());
            telemetry.addData("motor2 speed", flywheelMotor2.getCorrectedVelocity());
            telemetry.addData("motor1 pos", flywheelMotor1.getCurrentPosition());
            telemetry.addData("motor2 pos", flywheelMotor2.getCurrentPosition());
            telemetry.update();
        }
    }
}
