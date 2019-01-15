package org.firstinspires.ftc.teamcode.FTC8582;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by myradada on 10/9/17.
 */

@TeleOp(name = "TeleOp8582", group = "TeleOp")
//@Disabled
public class TeleOp8582 extends LinearOpMode {

    //Declare OpMode members
    Hardware8582 robot = new Hardware8582();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);

        //constants (change as needed)
        double OPEN_LEFT = 0.8;     //left servo open position
        final double CLOSE_LEFT = 1.0;   //left servo closed position
        double OPEN_RIGHT = 0.2;    //right servo open position
        final double CLOSE_RIGHT = 0.0;   //right servo closed position
        final double INIT_LEFT = 0.0;
        final double INIT_RIGHT = 1.0;
        final double UP_JEWELS = 0.2;     //jewel servo up position
        final double DOWN_JEWELS = 0.8;   //jewel servo down position (in teleop, lower for autonomous)

        boolean clawPrevState = false;    //claw servos previous state variable
        boolean clawCurrState = false;    //claw servos current state variable
        boolean clawServo = false;        //claw servos is in open position

        boolean openPrevState = false;
        boolean openCurrState = false;
        boolean openPosition = false;

        boolean jsPrevState = false;      //jewel servo previous state variable
        boolean jsCurrState = false;      //jewel servo current state variable
        boolean jewelServo = false;       //jewel servo is in down position


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.update();

            clawCurrState = gamepad2.a;
            jsCurrState = gamepad2.right_bumper;
            openCurrState = gamepad2.left_bumper;

            robot.motorLift.setPower(-gamepad2.left_stick_y);

            //arcade drive
            double x = gamepad1.left_stick_x;        //x direction (strafing)
            double y = -gamepad1.left_stick_y;       //y direction (forward/backward)
            double r = gamepad1.right_stick_x;       //rotation (left = counterclockwise)

            /*double frontRight = (y + x - r); JEF
            double frontLeft = (y + x + r);
            double backRight = (y - x - r);
            double backLeft = (y - x + r); */

            double frontRight = (y - x - r); //old- worked (check wiring)
            double frontLeft = (y + x + r);
            double backRight = (y + x - r);
            double backLeft = (y - x + r);

            /*double frontRight = (y - x - r);
            double frontLeft = (y + (x/2) + r);
            double backRight = (-y + (x/2) - r);
            double backLeft = (-y - x + r); */

            frontRight = Range.clip(frontRight, -1, 1);
            frontLeft = Range.clip(frontLeft, -1, 1);
            backRight = Range.clip(backRight, -1, 1);
            backLeft = Range.clip(backLeft, -1, 1);

            robot.motorFR.setPower(frontRight);
            robot.motorFL.setPower(frontLeft);
            robot.motorBR.setPower(backRight);
            robot.motorBL.setPower(backLeft);


            if ((clawCurrState) && (clawCurrState != clawPrevState)) {   //button is transitioning to a pressed state
                clawServo = !clawServo;  //variable goes from false to true, so the servos should close
            }
            // update previous state variable.
            clawPrevState = clawCurrState;
            if (clawServo) {
                robot.rightClaw.setPosition(CLOSE_RIGHT);   //set servos to closed position
                robot.leftClaw.setPosition(CLOSE_LEFT);
            } else {
                robot.rightClaw.setPosition(OPEN_RIGHT); //set servos to open position
                robot.leftClaw.setPosition(OPEN_LEFT);
            }


            if ((openCurrState) && (openCurrState != openPrevState)) {
                openPosition = !openPosition;
            }
            openPrevState = openCurrState;
            if (openPosition) {
                OPEN_LEFT = 0.8;
                OPEN_RIGHT = 0.2;
            } else {
                OPEN_LEFT = 0.0;
                OPEN_RIGHT = 1.0;
            }


            if ((jsCurrState) && (jsCurrState != jsPrevState)) {   //button is transitioning to a pressed state
                jewelServo = !jewelServo;  //variable goes from false to true, so the servo should move
            }
            // update previous state variable.
            jsPrevState = jsCurrState;
            if (jewelServo) {
                robot.jewels.setPosition(DOWN_JEWELS);  //set servo to open position
            } else {
                robot.jewels.setPosition(UP_JEWELS); //set servo to closed position
            }


            telemetry.addData("Status", "Run Time: " + String.format("%.2s", runtime.toString()));
            telemetry.addData("rangeLeft", robot.rangeLeft.getDistance(DistanceUnit.INCH));
            telemetry.addData("rangeRight", robot.rangeRight.getDistance(DistanceUnit.INCH));
            telemetry.update();

            idle();
        }
    }
}
