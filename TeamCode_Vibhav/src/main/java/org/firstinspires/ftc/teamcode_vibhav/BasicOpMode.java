/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode_vibhav;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Basic_OpMode: (Linear)", group="Linear OpMode")
// @Disabled
public class BasicOpMode extends LinearOpMode {
    /** Inherit OpMode members from the Hardware Class. */
    Hardware hw = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /** Wait for the driver to press 'PLAY' (In DC) */
        waitForStart();
        runtime.reset();

        /** Run until driver presses 'STOP' (In DC) */
        while (opModeIsActive()) {
            /** Setup a variable for each drive wheel to save power level for telemetry */
            double leftPower;
            double rightPower;

            /** POV Mode uses left stick to go forward, and right stick to turn.
            * ~ This uses basic math to combine motions and is easier to drive straight. */
            double drive = -gamepad1.left_stick_y;
            double turn =  gamepad1.right_stick_x;
            leftPower = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower = Range.clip(drive - turn, -1.0, 1.0) ;

            /** Send calculated power to wheels */
            hw.leftFront.setPower(leftPower);
            hw.rightFront.setPower(rightPower);
            hw.leftBack.setPower(rightPower);
            hw.rightBack.setPower(leftPower);

            /** Show the elapsed game time and wheel power. */
            telemetry.addData("Status, ", "Run Time: " + runtime.toString());
            telemetry.addData("Motors, ", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}