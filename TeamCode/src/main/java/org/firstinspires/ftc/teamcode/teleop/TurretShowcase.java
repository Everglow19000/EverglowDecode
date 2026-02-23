package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.LazyHardwareMapImu;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class TurretShowcase extends LinearOpMode {
    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;
    class IMULocalizer implements Localizer {
        public final LazyImu lazyImu;

        Rotation2d yawOffset;

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
    @Override
    public void runOpMode() throws InterruptedException {
        turret = new Turret(hardwareMap);
        localizer = new IMULocalizer(hardwareMap);

        waitForStart();

        Actions.runBlocking(turret.getHomeTurretAction());

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                wantedGlobalTurretRotation = new Rotation2d(gamepad1.right_stick_x, gamepad1.right_stick_x);
            }

            turret.setGlobalAngle(wantedGlobalTurretRotation, localizer.getPose().heading);

            telemetry.addData("right y", gamepad1.right_stick_y);
            telemetry.addData("right x", gamepad1.right_stick_x);
            telemetry.addData("global wanted turret rotation", wantedGlobalTurretRotation.toDouble());
            telemetry.update();
        }
    }
}
