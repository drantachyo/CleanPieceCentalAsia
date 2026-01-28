package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Claw {
    private final Servo servo;

    // Константы позиций. Их удобно настраивать через Dashboard.
    public static double OPEN_POSITION = 0.5;
    public static double CLOSE_POSITION = 0.1;

    public Claw(HardwareMap hw) {
        // Имя "claw" должно совпадать с конфигурацией в Driver Station
        servo = hw.get(Servo.class, "claw");
    }

    public void open() {
        servo.setPosition(OPEN_POSITION);
    }

    public void close() {
        servo.setPosition(CLOSE_POSITION);
    }

    // Полезный метод, чтобы знать текущую позицию
    public double getPosition() {
        return servo.getPosition();
    }
}