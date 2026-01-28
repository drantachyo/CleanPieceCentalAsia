package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SimpleTurret;

@TeleOp(name = "Test: Simple Turret", group = "Test")
public class TestTeleOp extends OpMode {

    private Follower follower;
    private SimpleTurret turret;
    private TelemetryManager telemetryM;

    // Переменная для ручного изменения угла бамперами
    private double manualTarget = 0;

    @Override
    public void init() {
        // Инициализация Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8, 8, 0));

        // Инициализация турели
        turret = new SimpleTurret(hardwareMap);

        // Твоя телеметрия через панельки
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // 1. Управление базой (Pedro)
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
        follower.update();

        // 2. Логика выбора угла для турели
        if (gamepad1.a) {
            manualTarget = 0; // В центр
        } else if (gamepad1.b) {
            manualTarget = Math.toRadians(90); // Направо
        } else if (gamepad1.x) {
            manualTarget = Math.toRadians(-90); // Налево
        }

        // Плавная доводка бамперами (удерживаешь — угол меняется)
        if (gamepad1.right_bumper) manualTarget += 0.02;
        if (gamepad1.left_bumper)  manualTarget -= 0.02;

        // Отправляем команду в "абсолютную" функцию
        turret.setTargetAngle(manualTarget);

        // Обновляем моторы
        turret.updateManual();

        // 3. Вывод данных для настройки PID в реальном времени
        telemetryM.debug("Turret/Target Deg", Math.toDegrees(manualTarget));
        telemetryM.debug("Turret/Current Deg", Math.toDegrees(turret.getCurrentAngle()));
        telemetryM.debug("Turret/Power", "N/A"); // Если хочешь, можно добавить геттер мощности в класс

        telemetryM.update();
    }
}