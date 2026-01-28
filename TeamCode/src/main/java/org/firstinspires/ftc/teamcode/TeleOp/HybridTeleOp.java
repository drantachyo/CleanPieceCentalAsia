package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.SimpleTurret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@TeleOp(name = "Hybrid Turret Test")
public class HybridTeleOp extends OpMode {
    private Follower follower;
    private SimpleTurret turret;
    private Vision vision;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        turret = new SimpleTurret(hardwareMap);
        vision = new Vision(hardwareMap);
    }

    @Override
    public void loop() {
        follower.update();
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);

        // Одна кнопка для всего
        if (gamepad1.right_trigger > 0.5) {
            // Only command tracking if we aren't already doing it
            if (turret.getState() == SimpleTurret.State.IDLE) {
                turret.track( 21, 144.0, 144.0); // Ensure target coordinates are correct for the season!
            }
        } else {
            turret.idle();
        }

        // Башня сама решит, использовать vision или follower.getPose()
        turret.update(follower.getPose(), vision);

        telemetry.addData("Turret State", turret.getState());
        telemetry.addData("Angle", Math.toDegrees(turret.getCurrentAngle()));
        telemetry.update();
    }
}