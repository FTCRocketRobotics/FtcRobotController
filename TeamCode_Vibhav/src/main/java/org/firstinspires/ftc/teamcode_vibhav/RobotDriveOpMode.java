package org.firstinspires.ftc.teamcode_vibhav;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@TeleOp
public class RobotDriveOpMode extends LinearOpMode {
    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;
    public DcMotor bR;
    static final double COUNTS_PER_MOTOR_REV_GOBUILDA = 383.06;    // eg: GoBuilda Motor Encoder
    static final double COUNTS_PER_MOTOR_REV_ANDYMARK = 1120.00;    // eg: AndyMark Neverest 40
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");

        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.REVERSE);
        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.REVERSE);

        //Wait for driver to press PLAY
        waitForStart();

        //run until the driver presses STOP
        while (opModeIsActive()) {
            // We are thinking of the joysticks as a graph with four quadrants.
            double xAxis = gamepad1.left_stick_x;
            double yAxis = gamepad1.left_stick_y;
            double spinningInCircle = gamepad1.right_stick_x;

            double directionOfTravel;
            double direction;

            double fL_power;
            double fR_power;
            double bL_power;
            double bR_power;

            directionOfTravel = Math.atan2(yAxis, xAxis);
            direction = directionOfTravel - Math.PI / 4;

            // The Robots front two wheels are get half the power as the back, so we are x2 it.
            fL_power = (Math.cos(direction) - spinningInCircle) * 2;
            fR_power = (Math.sin(direction) + spinningInCircle) * 2;
            bL_power = (Math.sin(direction) - spinningInCircle) * 2;
            bR_power = (Math.cos(direction) + spinningInCircle) * 2;

            // Math.abs, is a class in JDK that will get the absolute value of a number
            if (Math.abs(fL_power) < 1.0 && Math.abs(fR_power) < 1.0 && Math.abs(bL_power) < 1.0 && Math.abs(bR_power) < 1.0) {
                List<Double> powers = new ArrayList<Double>();
                double largestPowers = Collections.max(powers);

                powers.add(Math.abs(fL_power));
                powers.add(Math.abs(fR_power));
                powers.add(Math.abs(bL_power));
                powers.add(Math.abs(bR_power));

                fL_power = fL_power / largestPowers;
                fR_power = fR_power / largestPowers;
                bL_power = bL_power / largestPowers;
                bR_power = bR_power / largestPowers;
            }
            if ((xAxis == 0 && yAxis == 0) || gamepad1 == null) {
                fL.setPower(Math.pow(-spinningInCircle, 3));
                fR.setPower(Math.pow(spinningInCircle, 3));
                bL.setPower(Math.pow(-spinningInCircle, 3));
                bR.setPower(Math.pow(spinningInCircle, 3));
            } else {
                fL.setPower(Math.pow(fL_power, 3));
                fR.setPower(Math.pow(fR_power, 3));
                bL.setPower(Math.pow(bL_power, 3));
                bR.setPower(Math.pow(bR_power, 3));
            }
            telemetry.addData("Front  Left:", fL_power);
            telemetry.addData("Front Right:", fR_power);
            telemetry.addData("Back   Left:", bL_power);
            telemetry.addData("Back  Right:", bR_power);
            telemetry.update();
        }
    }
}
/** © All Rights Reserved */