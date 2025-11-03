package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.EmbeddedControlHubModule;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

@TeleOp(name = "ShooterTuningFormula")
@Config
public class ShooterTuning extends LinearOpMode {

    public static double minPosition = 0.35;
    public static double maxPosition = 0.62;
    public static boolean servoFlipped = true;
    public static double currPos = 0;
    public static double wantedVoltage = 0;

    DcMotorEx leftMotor;
    Servo hoodController;
    LynxModule controlHub;

    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        hoodController = hardwareMap.get(Servo.class, "hoodServo");
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        leftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        GamepadEx gp = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            gp.readButtons();

            hoodController.scaleRange(minPosition, maxPosition);

            hoodController.setPosition(currPos);
            hoodController.setDirection(servoFlipped ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);

            leftMotor.setPower(wantedVoltage/controlHub.getInputVoltage(VoltageUnit.VOLTS));

            telemetry.addData("current", leftMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("motor power", leftMotor.getPower());
            telemetry.addData("servo pos", hoodController.getPosition());
            telemetry.update();
        }
    }
}