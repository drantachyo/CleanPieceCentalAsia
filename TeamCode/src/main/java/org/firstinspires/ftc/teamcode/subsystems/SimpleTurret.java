package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.math.MathUtils;
import org.firstinspires.ftc.teamcode.math.PIDFController;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Configurable
public class SimpleTurret {
    public enum State { IDLE, SEARCHING, LOCKED }
    private State currentState = State.IDLE;

    private final DcMotorEx motor;
    private final PIDFController controller;

    public static double kP = 0.5, kI = 0, kD = 0, kS = 0;
    public static double MIN_LIMIT = Math.toRadians(-90), MAX_LIMIT = Math.toRadians(90);
    public static double TICKS_PER_RADIAN = (28.0 * 5.0) / (2 * Math.PI);

    private double targetAngle = 0;
    private int activeTagId = -1;
    private double tx, ty;

    public SimpleTurret(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "turret");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        controller = new PIDFController(kP, kI, kD, 0, kS);
    }

    // Включает режим авто-слежения
    public void track(int id, double x, double y) {
        this.activeTagId = id;
        this.tx = x;
        this.ty = y;
        // Если мы были в IDLE, начинаем поиск
        if (currentState == State.IDLE) currentState = State.SEARCHING;
    }

    // Сброс в нейтральное положение
    public void idle() {
        this.currentState = State.IDLE;
        this.activeTagId = -1;
        setTargetAngle(0);
    }

    // Вся магия переключений здесь
    public void update(Pose robotPose, Vision vision) {
        controller.setPIDF(kP, kI, kD, 0, kS);

        switch (currentState) {
            case IDLE:
                // Сидим в 0, ждем команду track()
                break;

            case SEARCHING:
                // 1. Наводимся по одометрии
                applyOdometry(robotPose);
                // 2. Если вдруг увидели тег — фиксируем (LOCKED)
                if (vision.getTarget(activeTagId) != null) currentState = State.LOCKED;
                break;

            case LOCKED:
                AprilTagDetection tag = vision.getTarget(activeTagId);
                if (tag != null) {
                    // 1. Точная доводка по камере
                    double bearingRad = Math.toRadians(tag.ftcPose.bearing);
                    setTargetAngle(getCurrentAngle() + bearingRad);
                } else {
                    // 2. Потеряли тег — откатываемся на одометрию
                    currentState = State.SEARCHING;
                }
                break;
        }

        // Исполнение PID
        double current = getCurrentAngle();
        double error = MathUtils.normalizeAngle(targetAngle - current);
        double power = controller.calculate(error);

        if (current >= MAX_LIMIT && power > 0) power = 0;
        if (current <= MIN_LIMIT && power < 0) power = 0;
        motor.setPower(power);
    }
    public void updateManual(){
        controller.setPIDF(kP, kI, kD, 0, kS);

        double currentAngle = getCurrentAngle();
        double error = MathUtils.normalizeAngle(targetAngle - currentAngle);

        double power = controller.calculate(error);

        if (currentAngle > MAX_LIMIT && power > 0) power = 0;
        if (currentAngle < MIN_LIMIT && power < 0) power = 0;

        motor.setPower(power);
    }

    private void applyOdometry(Pose robotPose) {
        double dx = tx - robotPose.getX();
        double dy = ty - robotPose.getY();
        double relAngle = Math.atan2(dy, dx) - robotPose.getHeading();
        setTargetAngle(relAngle);
    }

    public void setTargetAngle(double rad) {
        this.targetAngle = Range.clip(MathUtils.normalizeAngle(rad), MIN_LIMIT, MAX_LIMIT);
    }

    public double getCurrentAngle() { return motor.getCurrentPosition() / TICKS_PER_RADIAN; }
    public State getState() { return currentState; }
}