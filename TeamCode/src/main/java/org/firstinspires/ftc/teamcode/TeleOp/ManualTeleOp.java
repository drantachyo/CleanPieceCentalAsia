package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.SimpleTurret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@TeleOp(name = "Full Robot TeleOp", group = "Main")
public class ManualTeleOp extends OpMode {

    // Подсистемы
    private Follower follower;
    private SimpleTurret turret;
    private Vision vision;
    private Shooter shooter;
    private Intake intake;
    private Claw claw;

    @Override
    public void init() {
        // 1. Движение (Pedro Pathing)
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(8, 8, 0)); // Настрой начальную позу под свои нужды

        // 2. Турель и Зрение
        turret = new SimpleTurret(hardwareMap);
        vision = new Vision(hardwareMap);

        // 3. Механизмы
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        claw = new Claw(hardwareMap);

        telemetry.addLine("Робот готов! Жми Start.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // --- 1. УПРАВЛЕНИЕ БАЗОЙ (Pedro Pathing) ---
        // Левый стик - движение, Правый стик (X) - поворот
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false // Робо-центричное управление (поставь true для Field-Centric)
        );
        follower.update();

        // --- 2. УПРАВЛЕНИЕ ТУРЕЛЬЮ (Auto-Tracking) ---
        // Если зажат правый триггер — следим за AprilTag (ID 5) по координатам (144, 144)
        if (gamepad1.right_trigger > 0.1) {
            turret.track(21, 144.0, 144.0);
        } else {
            turret.idle(); // Иначе возвращаемся в 0
        }
        // Обновляем состояние турели (передаем текущую позу и зрение)
        turret.update(follower.getPose(), vision);

        // --- 3. УПРАВЛЕНИЕ ШУТЕРОМ (Velocity PID) ---
        if (gamepad1.y) {
            shooter.setTargetRPM(3000); // Запуск на рабочую скорость
        } else if (gamepad1.x) {
            shooter.setTargetRPM(0);    // Остановка
        }
        shooter.update();

        // --- 4. УПРАВЛЕНИЕ ИНТЕЙКОМ ---
        if (gamepad1.right_bumper) {
            intake.intake();
        } else if (gamepad1.left_bumper) {
            intake.outtake();
        } else {
            intake.stop();
        }

        // --- 5. УПРАВЛЕНИЕ КЛЕШНЕЙ ---
        if (gamepad1.a) {
            claw.open();
        } else if (gamepad1.b) {
            claw.close();
        }

        // --- ТЕЛЕМЕТРИЯ ---
        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addData("--- Turret ---", "");
        telemetry.addData("State", turret.getState());
        telemetry.addData("Angle (Deg)", Math.toDegrees(turret.getCurrentAngle()));

        telemetry.addData("--- Shooter ---", "");
        telemetry.addData("Current RPM", (int)shooter.getCurrentRPM());
        telemetry.addData("Ready", shooter.isReady() ? "YES" : "WAIT");

        telemetry.addData("--- Pose ---", "");
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();
    }

    @Override
    public void stop() {
        // Останавливаем тяжелые процессы
        vision.stop();
    }
}