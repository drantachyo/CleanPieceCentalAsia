package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable
public class Shooter {
    private final DcMotorEx motor;

    // --- КОНСТАНТЫ МОТОРА ---
    // Например, для GoBilda 1:1 это 28, для Matrix/Rev другие значения.
    // Проверь спецификацию своего мотора!
    public static double TICKS_PER_REV = 28.0;
    public static double MAX_RPM = 6000.0; // Примерный предел мотора

    // Допуск (когда считаем, что шутер готов стрелять)
    public static double RPM_TOLERANCE = 100;

    // Коэффициенты PIDF для встроенного контроллера
    // F (Feedforward) - самый важный для удержания скорости
    // P (Proportional) - для реакции на просадку скорости при выстреле
    public static double kP = 50.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 12.0; // Нужно подбирать! (см. гайд ниже)

    private double targetRPM = 0;

    public Shooter(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "shooter");

        // ВАЖНО: Режим RUN_USING_ENCODER включает встроенный Velocity PID
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // FLOAT лучше для шутера, чтобы не тормозил резко

        // Применяем настройки PIDF при инициализации
        setPIDF(kP, kI, kD, kF);
    }

    public void update() {
        // Конвертируем RPM в Тики/Сек
        // (RPM / 60) * TICKS_PER_REV
        double targetTicksPerSec = (targetRPM / 60.0) * TICKS_PER_REV;

        // Отправляем команду скорости на Hub
        motor.setVelocity(targetTicksPerSec);
    }

    // Метод для обновления коэффициентов (удобно для настройки через Dashboard)
    public void setPIDF(double p, double i, double d, double f) {
        PIDFCoefficients coeffs = new PIDFCoefficients(p, i, d, f);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
    }

    public void setTargetRPM(double rpm) {
        // Ограничиваем, чтобы не перегреть мотор, если попросим 100 000 RPM
        if (rpm > MAX_RPM) rpm = MAX_RPM;
        this.targetRPM = rpm;
    }

    public double getCurrentRPM() {
        return (motor.getVelocity() / TICKS_PER_REV) * 60.0;
    }

    public boolean isReady() {
        return Math.abs(targetRPM - getCurrentRPM()) < RPM_TOLERANCE && targetRPM > 100;
    }
}