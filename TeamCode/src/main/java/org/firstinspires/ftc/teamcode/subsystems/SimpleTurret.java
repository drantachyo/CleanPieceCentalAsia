package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.math.PIDFController;

@Configurable
public class SimpleTurret {
    private final DcMotorEx motor;
    private final PIDFController controller;

    // Константы для настройки (в панельках)
    public static double kP = 0.5, kI = 0, kD = 0, kS = 0;
    public static double MIN_LIMIT = Math.toRadians(-90), MAX_LIMIT = Math.toRadians(90);

    // Один раз настраиваем под мотор
    public static double TICKS_PER_REV = 145.1;
    public static double GEAR_RATIO = 1.0;
    private final double TICKS_PER_RADIAN = (TICKS_PER_REV * GEAR_RATIO) / (2 * Math.PI);

    private double targetAngle = 0;

    public SimpleTurret(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, "turret");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new PIDFController(kP, kI, kD, 0, kS);
    }

    /**
     * Абсолютная функция установки цели.
     * Сюда приходят данные и от вижна, и от одометрии.
     */
    public void setTargetAngle(double rad) {
        // Ограничиваем угол, чтобы не порвать провода
        this.targetAngle = Range.clip(rad, MIN_LIMIT, MAX_LIMIT);
    }

    public void update() {
        // Синхронизируем PID (если крутил ползунки в панельке)
        controller.setPIDF(kP, kI, kD, 0, kS);

        double currentAngle = getCurrentAngle();
        double power = controller.calculate(targetAngle - currentAngle);

        // Финальный предохранитель: не даем мотору давить в лимит
        if (currentAngle >= MAX_LIMIT && power > 0) power = 0;
        if (currentAngle <= MIN_LIMIT && power < 0) power = 0;

        motor.setPower(power);
    }

    public double getCurrentAngle() {
        return motor.getCurrentPosition() / TICKS_PER_RADIAN;
    }
}