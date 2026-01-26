package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable; // Твой импорт
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.math.PIDFController;

@Configurable
public class SimpleTurret {
    private DcMotorEx motor;
    private PIDFController controller;

    // === НАСТРОЙКИ (Меняются в панельке) ===
    // Проверь свои тики! (Gobilda 5202 = 145.1, 5203 = 384.5 и т.д.)
    public static double TICKS_PER_REV_MOTOR = 145.1;
    public static double GEAR_RATIO = 1.0;

    // PID
    public static double kP = 0.5;
    public static double kI = 0.0;
    public static double kD = 0.001;
    public static double kF = 0.0;
    public static double kS = 0.0; // Важно для старта движения

    // Лимиты (Радианы)
    public static double MIN_LIMIT = Math.toRadians(-90);
    public static double MAX_LIMIT = Math.toRadians(90);

    // Расчетная константа (не для конфига, но нужна классу)
    private double ticksPerRadian;

    private double targetAngle = 0;

    public SimpleTurret(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "turret");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Предварительный расчет, чтобы не считать каждый цикл
        updateConstants();

        controller = new PIDFController(kP, kI, kD, kF, kS);
    }

    public void update() {
        // Если меняешь GEAR_RATIO на лету, лучше пересчитывать это тут
        updateConstants();

        // Обновляем PID из панельки
        controller.setPIDF(kP, kI, kD, kF, kS);

        double currentAngle = getCurrentAngle();
        double error = targetAngle - currentAngle;

        // Если провода позволяют крутиться бесконечно, раскомментируй normalizeAngle
        // Но для проводной турели лучше без нормализации, но с лимитами

        double power = controller.calculate(error);

        // Хард-стоп софтвером (двойная защита)
        if (currentAngle > MAX_LIMIT && power > 0) power = 0;
        if (currentAngle < MIN_LIMIT && power < 0) power = 0;

        motor.setPower(power);
    }

    private void updateConstants() {
        ticksPerRadian = (TICKS_PER_REV_MOTOR * GEAR_RATIO) / (2 * Math.PI);
    }

    public void setTargetAngle(double rad) {
        // Лимитируем цель, чтобы не просить невозможного
        this.targetAngle = Range.clip(rad, MIN_LIMIT, MAX_LIMIT);
    }

    public double getCurrentAngle() {
        if (ticksPerRadian == 0) return 0;
        return motor.getCurrentPosition() / ticksPerRadian;
    }

    public double getTargetAngle() { return targetAngle; }
    public double getError() { return targetAngle - getCurrentAngle(); }
}