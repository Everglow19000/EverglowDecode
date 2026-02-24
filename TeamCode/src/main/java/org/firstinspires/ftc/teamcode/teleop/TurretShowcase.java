package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name="Turret Showcase", group="Driving")
@Config
public class TurretShowcase extends LinearOpMode {
    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;
    class IMULocalizer implements Localizer {
        public final LazyImu lazyImu;

        Rotation2d yawOffset = Rotation2d.exp(0);

        public IMULocalizer(HardwareMap hardwareMap) {
            lazyImu = new LazyHardwareMapImu(hardwareMap, "imu", new RevHubOrientationOnRobot(
                    logoFacingDirection, usbFacingDirection));
            lazyImu.get().resetYaw();
        }
        @Override
        public void setPose(Pose2d pose) {
            yawOffset = Rotation2d.exp(pose.heading.toDouble() - lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        }

        @Override
        public Pose2d getPose() {
            return new Pose2d(new Vector2d(0, 0), yawOffset.toDouble() + lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        }

        @Override
        public PoseVelocity2d update() {
            return new PoseVelocity2d(new Vector2d(0, 0), 0);
        }
    }
    Turret turret;
    IMULocalizer localizer;
    Rotation2d wantedGlobalTurretRotation = Rotation2d.exp(0);
    Vector2d joystickVector = new Vector2d(0, 0);
    public static boolean manual = true;
    public static int motorTicksWanted = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        turret = new Turret(hardwareMap);
        localizer = new IMULocalizer(hardwareMap);

        boolean circleFlag = false;

        waitForStart();

        Actions.runBlocking(turret.getHomeTurretAction());

        while (opModeIsActive()) {
            if (gamepad1.circle && !circleFlag) {
                manual = true;
            }
            circleFlag = gamepad1.circle;

            if (manual) {
                if (gamepad1.right_bumper) {
                    joystickVector = new Vector2d(gamepad1.right_stick_x, -gamepad1.right_stick_y);
                    Vector2d normalized = new Vector2d(joystickVector.x, joystickVector.y);
                    normalized.norm();
                    wantedGlobalTurretRotation = new Rotation2d(normalized.x, normalized.y);
                }

                turret.setGlobalAngle(wantedGlobalTurretRotation, localizer.getPose().heading);

                telemetry.addData("x", joystickVector.x);
                telemetry.addData("y", joystickVector.y);
                if (wantedGlobalTurretRotation != null) {
                    telemetry.addData("global wanted rotation", wantedGlobalTurretRotation.toDouble());
                }
                else {
                    telemetry.addLine("rotation is null!");
                }
            }
            else {
                turret.turretMotor.setTargetPosition(motorTicksWanted);
            }
            telemetry.update();
        }
    }
}
