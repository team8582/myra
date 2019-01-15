package org.firstinspires.ftc.teamcode.FTC8582;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by myradada on 10/17/17.
 */

@Autonomous(name = "RedRightGlyph", group = "Autonomous")
//@Disabled
public class RedRightGlyph extends LinearOpMode {

    Hardware8582 robot = new Hardware8582();
    private ElapsedTime runtime = new ElapsedTime();       //Create variable to keep track of elapsed time.
    static final double DRIVE_SPEED = 0.7;
    static final double TURN_SPEED = 0.5;
    static final double STRAFE_SPEED = 0.5;

    final double OPEN_LEFT = 0.5;     //left servo open position
    final double CLOSE_LEFT = 0.94;   //left servo closed position
    final double OPEN_RIGHT = 1.0;    //right servo open position
    final double CLOSE_RIGHT = 0.5;   //right servo closed position
    final double INIT_LEFT = 0.0;
    final double INIT_RIGHT = 1.0;
    final double UP_JEWELS = 0.2;     //release servo open position
    final double DOWN_JEWELS = 0.9;   //release servo closed position

    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //robot.colorSensor.enableLed(true);

        robot.mrGyro.calibrate();

        robot.motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdVHv4L/////AAAAGWYErZ5b3kYSm+I+BH8WGV8293k4zGyjzQysjcJiURTRHPRpfJH44ca+HJYLrsPhg8SDfAhNMF5uFt1n4RxQqn66Yhm9na/eJ2F67MUTXco0bu5XbV4qiSyzhEmTx02gIzPRrJBVQdEzECoz/AX841H8H+KMBnJxlIXLjdQS0eTHCDG5nEheXua5A4PK8MaKRfp3RoETJEHPfpz2bdFBhAwwa4XJFJ+m0v9/M8N2Mbhla5yS36CtLlcE+/LrVV+PavhbK6PYm1RC3rJ74IKPxfoTNFo4Izaz3OwPf6UUZ7B/tzASRgSSqBKrR941auZzR5ZQGz47sm8DQFgdmxS9ogCx7Q7Xil6sUTpqG1MzDK+y";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //robot.leftClaw.setPosition(CLOSE_LEFT);
        //robot.rightClaw.setPosition(CLOSE_RIGHT);

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        telemetry.addData("VuMark", "%s visible", vuMark);

        encoderDrive(DRIVE_SPEED, -25, -25, 10);
        turnAbsolute(180);

        if (vuMark == RelicRecoveryVuMark.LEFT) {
            strafeUsingRight(-18, 10);
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            strafeUsingRight(-4, 5);
        } else {
            strafeUsingRight(-10, 10);
        }

        encoderDrive(DRIVE_SPEED, 6, 6, 5);

        //robot.leftClaw.setPosition(OPEN_LEFT);
        //robot.rightClaw.setPosition(OPEN_RIGHT);

        idle();
    }

    // Method for driving a set distance using encoders:
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) throws InterruptedException {
        int newLeftTarget; //declare variable for the target position of the left motor
        int newRightTarget; //declare variable for the target position of the right motor

        final double COUNTS_PER_MOTOR_REV = 1120;     //Andymark Motor Encoder
        final double DRIVE_GEAR_REDUCTION = 1.0;      //This is < 1.0 if geared up
        final double WHEEL_DIAMETER_INCHES = 4.0;     //For figuring circumference
        final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) 
                (WHEEL_DIAMETER_INCHES * 3.1415);    //calculates wheel rotations necessary to move 1 in

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorFL.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH); //initializes newLeftTarget to represent the desired distance travelled by the left motors
            newRightTarget = robot.motorFR.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH); //initializes newRightTarget to represent the desired distance travelled by the right motors
            
            //sets the motors to run to the position corresponding to their side (left or right)
            robot.motorFR.setTargetPosition(newRightTarget); 
            robot.motorBR.setTargetPosition(newRightTarget);
            robot.motorFL.setTargetPosition(newLeftTarget);
            robot.motorBL.setTargetPosition(newLeftTarget);

            // Turn On RUN_TO_POSITION
            robot.motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.motorFR.setPower(Math.abs(speed));
            robot.motorFL.setPower(Math.abs(speed));
            robot.motorBR.setPower(Math.abs(speed));
            robot.motorBL.setPower(Math.abs(speed));

            // keep looping while still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorFR.isBusy() && robot.motorFL.isBusy() && robot.motorBR.isBusy() && robot.motorBL.isBusy())) {

                // Display postion on path to the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.motorFL.getCurrentPosition(),
                        robot.motorFR.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.motorFR.setPower(0);
            robot.motorFL.setPower(0);
            robot.motorBR.setPower(0);
            robot.motorBL.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(200);   // optional pause after each move
        }
    }


    //Method for turning a set amount using the MR Gyro Sensor:
     public void turnAbsolute(int target) throws InterruptedException {
        if (!opModeIsActive()) { //ensures opMode is on
            return;
        }

        while (opModeIsActive()) {

            int zAccumulated = robot.mrGyro.getIntegratedZValue(); //gets the current orientation of the robot with the starting point as 0 degrees

            while (opModeIsActive() && (Math.abs(zAccumulated - target) > 2)) {    //Turn includes a margin of error

                if (zAccumulated < target) {     //turns the robot to the left
                    robot.motorFR.setPower(TURN_SPEED);
                    robot.motorBR.setPower(TURN_SPEED);
                    robot.motorFL.setPower(-TURN_SPEED);
                    robot.motorBL.setPower(-TURN_SPEED);
                }

                if (zAccumulated > target) { //turns the robot to the right
                    robot.motorFR.setPower(-TURN_SPEED);
                    robot.motorBR.setPower(-TURN_SPEED);
                    robot.motorFL.setPower(TURN_SPEED);
                    robot.motorBL.setPower(TURN_SPEED);
                }

                zAccumulated = robot.mrGyro.getIntegratedZValue(); //accounts for the change in the robot's orientation as it turns
                telemetry.addData("1. accu", zAccumulated); //allows driver to track the turn's progress
                telemetry.update();

            }
            //stops the robot once the desired position has been reached
            robot.motorFR.setPower(0); 
            robot.motorBR.setPower(0);
            robot.motorFL.setPower(0);
            robot.motorBL.setPower(0);

            //displays information to the driver throughout method implementation
            telemetry.addData("1. accu", zAccumulated);
            telemetry.addData("2. target", target);
            telemetry.update();

            break;
        }

    }


    //Method for turning a set amount using the MR Gyro Sensor (accumulates more error than turnAbsolute):
     public void turnRelative(int target) throws InterruptedException {
        if (!opModeIsActive()) //makes sure opMode is on
            return;
        turnAbsolute(target + robot.mrGyro.getIntegratedZValue()); //calls turnAbsolute but modifies the parameter to make the robot's starting position or 0 degrees the robot's orientation at the time of the method call
    }

    //strafe using left range sensor
    public void strafeUsingLeft(int distance, int time) throws InterruptedException {
        if (!opModeIsActive()) { //make sure opMode is on
            return;
        }

        double currentDistance = robot.rangeLeft.getDistance(DistanceUnit.INCH); //declares and initializes a variable for the robot's starting position

        //move right
        if (distance > 0) {
            double target = currentDistance + distance; //declares and initializes the target position
            runtime.reset(); //reset runtime to make sure it doesn't interfere with the implementation of the time parameter

            //loop for strafing to the right until the desired position is reached
            while (opModeIsActive() && (currentDistance) < target && (runtime.seconds() < time)) {
                robot.motorFR.setPower(STRAFE_SPEED);
                robot.motorFL.setPower(STRAFE_SPEED);
                robot.motorBR.setPower(-STRAFE_SPEED);
                robot.motorBL.setPower(-STRAFE_SPEED);

                currentDistance = robot.rangeLeft.getDistance(DistanceUnit.INCH); //constantly reset the current position throughout the loop's running period

            
                //display progress to the driver
                telemetry.addData("leftRange: ", robot.rangeLeft.getDistance(DistanceUnit.INCH));
                telemetry.addData("target: ", target);
                telemetry.update();
            }
            
            //turns off motors once target has been achieved
            robot.motorFR.setPower(0);
            robot.motorFL.setPower(0);
            robot.motorBR.setPower(0);
            robot.motorBL.setPower(0);


        }

        //move left
        if (distance < 0) {
            double target = robot.rangeLeft.getDistance(DistanceUnit.INCH) + distance; //declares and initializes the target position 

            runtime.reset(); //reset runtime to make sure it doesn't interfere with the implementation of the time parameter

            //loop for strafing to the left until the desired position is reached
            while (opModeIsActive() && (currentDistance > target) && (runtime.seconds() < time)) {
                robot.motorFR.setPower(-STRAFE_SPEED);
                robot.motorFL.setPower(-STRAFE_SPEED);
                robot.motorBR.setPower(STRAFE_SPEED);
                robot.motorBL.setPower(STRAFE_SPEED);
                currentDistance = robot.rangeLeft.getDistance(DistanceUnit.INCH); //constantly reset the current position throughout the loop's running period

                //display progress to the driver
                telemetry.addData("leftRange: ", robot.rangeLeft.getDistance(DistanceUnit.INCH));
                telemetry.addData("target: ", target);
                telemetry.update();
            }
            //turns off motors once target has been achieved
            robot.motorFR.setPower(0);
            robot.motorFL.setPower(0);
            robot.motorBR.setPower(0);
            robot.motorBL.setPower(0);
        }

    }

    //strafe using right range sensor
  
    public void strafeUsingRight(int distance, int time) throws InterruptedException {
        if (!opModeIsActive()) { //ensure opMode is on
            return;
        }

        double currentDistance = robot.rangeRight.getDistance(DistanceUnit.INCH); //declares and initializes a variable for the robot's starting position


        //move right
        if (distance > 0) {
            double target = currentDistance - distance; //declares and initializes the target position
            runtime.reset(); //reset runtime to make sure it doesn't interfere with the implementation of the time parameter

            //loop for strafing to the right until the desired position is reached
            while (opModeIsActive() && (currentDistance > target) && (runtime.seconds() < time)) {
                robot.motorFR.setPower(STRAFE_SPEED);
                robot.motorFL.setPower(STRAFE_SPEED);
                robot.motorBR.setPower(-STRAFE_SPEED);
                robot.motorBL.setPower(-STRAFE_SPEED);

                currentDistance = robot.rangeRight.getDistance(DistanceUnit.INCH); //constantly reset the current position throughout the loop's running period

                //display progress to driver
                telemetry.addData("rightRange: ", robot.rangeRight.getDistance(DistanceUnit.INCH));
                telemetry.addData("target: ", target);
                telemetry.update();
            }
            //turn off motors once target has been achieved
            robot.motorFR.setPower(0);
            robot.motorFL.setPower(0);
            robot.motorBR.setPower(0);
            robot.motorBL.setPower(0);
        }

        //move left
        if (distance < 0) {
            double target = robot.rangeRight.getDistance(DistanceUnit.INCH) - distance; //declares and initializes the target position
            runtime.reset(); //eset runtime to make sure it doesn't interfere with the implementation of the time parameter

            //loop for strafing to the left until the desired position is reached
            while (opModeIsActive() && (currentDistance < target) && (runtime.seconds() < time)) {
                robot.motorFR.setPower(-STRAFE_SPEED);
                robot.motorFL.setPower(-STRAFE_SPEED);
                robot.motorBR.setPower(STRAFE_SPEED);
                robot.motorBL.setPower(STRAFE_SPEED);
                currentDistance = robot.rangeRight.getDistance(DistanceUnit.INCH); //constantly reset the current position throughout the loop's running period

                //display progess to the driver
                telemetry.addData("rightRange: ", robot.rangeRight.getDistance(DistanceUnit.INCH));
                telemetry.addData("target: ", target);
                telemetry.update();
            }
            //turn off motors once target has been achieved
            robot.motorFR.setPower(0);
            robot.motorFL.setPower(0);
            robot.motorBR.setPower(0);
            robot.motorBL.setPower(0);
        }

    }
    public void strafeUsingRight(int distance, int time) throws InterruptedException {
        if (!opModeIsActive()) { //ensure opMode is on
            return;
        }

        double currentDistance = robot.rangeRight.getDistance(DistanceUnit.INCH); //declares and initializes a variable for the robot's starting position


        //move right
        if (distance > 0) {
            double target = currentDistance - distance; //declares and initializes the target position
            runtime.reset(); //reset runtime to make sure it doesn't interfere with the implementation of the time parameter

            //loop for strafing to the right until the desired position is reached
            while (opModeIsActive() && (currentDistance > target) && (runtime.seconds() < time)) {
                robot.motorFR.setPower(STRAFE_SPEED);
                robot.motorFL.setPower(STRAFE_SPEED);
                robot.motorBR.setPower(-STRAFE_SPEED);
                robot.motorBL.setPower(-STRAFE_SPEED);

                currentDistance = robot.rangeRight.getDistance(DistanceUnit.INCH); //constantly reset the current position throughout the loop's running period

                //display progress to driver
                telemetry.addData("rightRange: ", robot.rangeRight.getDistance(DistanceUnit.INCH));
                telemetry.addData("target: ", target);
                telemetry.update();
            }
            //turn off motors once target has been achieved
            robot.motorFR.setPower(0);
            robot.motorFL.setPower(0);
            robot.motorBR.setPower(0);
            robot.motorBL.setPower(0);
        }

        //move left
        if (distance < 0) {
            double target = robot.rangeRight.getDistance(DistanceUnit.INCH) - distance; //declares and initializes the target position
            runtime.reset(); //eset runtime to make sure it doesn't interfere with the implementation of the time parameter

            //loop for strafing to the left until the desired position is reached
            while (opModeIsActive() && (currentDistance < target) && (runtime.seconds() < time)) {
                robot.motorFR.setPower(-STRAFE_SPEED);
                robot.motorFL.setPower(-STRAFE_SPEED);
                robot.motorBR.setPower(STRAFE_SPEED);
                robot.motorBL.setPower(STRAFE_SPEED);
                currentDistance = robot.rangeRight.getDistance(DistanceUnit.INCH); //constantly reset the current position throughout the loop's running period

                //display progess to the driver
                telemetry.addData("rightRange: ", robot.rangeRight.getDistance(DistanceUnit.INCH));
                telemetry.addData("target: ", target);
                telemetry.update();
            }
            //turn off motors once target has been achieved
            robot.motorFR.setPower(0);
            robot.motorFL.setPower(0);
            robot.motorBR.setPower(0);
            robot.motorBL.setPower(0);
        }

    }

}