package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry; // Твой импорт
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SimpleTurret;

@TeleOp(name = "Test: Chassis + Turret", group = "Test")
public class TestTeleOp extends OpMode {

    private Follower follower;
    private SimpleTurret turret;
    private TelemetryManager telemetryM;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,0));

        turret = new SimpleTurret(hardwareMap);

        // Твоя телеметрия
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // 1. Шасси (Pedro)
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
        follower.update();

        // 2. Тест Турели (Кнопки)
        if (gamepad1.a) turret.setTargetAngle(0);
        if (gamepad1.b) turret.setTargetAngle(Math.toRadians(45));
        if (gamepad1.x) turret.setTargetAngle(Math.toRadians(-45));

        // Ручной тюнинг (Триггеры для тонкой доводки)
        if (gamepad1.right_bumper) turret.setTargetAngle(turret.getTargetAngle() + 0.01);
        if (gamepad1.left_bumper)  turret.setTargetAngle(turret.getTargetAngle() - 0.01);

        turret.update();

        // 3. Вывод в Panels
        telemetryM.debug("Turret/Target", Math.toDegrees(turret.getTargetAngle()));
        telemetryM.debug("Turret/Current", Math.toDegrees(turret.getCurrentAngle()));
        telemetryM.debug("Turret/Error", Math.toDegrees(turret.getError()));

        telemetryM.update();
    }
}