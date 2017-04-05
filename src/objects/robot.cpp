#include "objects/robot.h"

#include "world/world.h"

#include "sl/sl.h"

#include "util/util.h"

#include "glm/glm.hpp"
#include "glm/gtc/random.hpp"
using namespace glm;

#include <iostream>
#include <iomanip>
#include <fstream>
using namespace std;

const float Robot::ROBOT_RADIUS_METERS = 5.0;															// robot radius in meters
const float Robot::ROBOT_MOTOR_SPEED = 50.0;															// robot motor speed

const float Robot::ROBOT_COLLISION_BOUNDS = ROBOT_RADIUS_METERS * 2.0;									// how close we can get to a robot without colliding against it
const float Robot::ROBOT_COLLISION_BOUNDS_SQUARED = ROBOT_COLLISION_BOUNDS * ROBOT_COLLISION_BOUNDS;	// same value above, squared, for faster distance checks

const int Robot::OVERCROWDED_FED_UP_TIME = 1500;						// how long after being overcrowded (after the group change time is elapsed)
																		// we will change circle groups (msec)
const int Robot::RE_EVAL_GROUP_TIME = 5000;								// how often we evaluate our circle group based on our distance to center

float Robot::orbitSegments[360];										// orbit distances for each segment we can travel along our formation

Robot::Robot(World *world, vec2 pos, float dir)
{
	init();

	this -> world = world;
	setPos(pos);
	setDir(dir);
}

Robot::~Robot() { }		// nothing owned to deallocate

void Robot::updateFormationShape(int angle, float distance)
{
	orbitSegments[angle] = distance;
}

float Robot::getFormationShape(int angle)
{
	return orbitSegments[angle];
}

void Robot::setupDefaultFormation()
{
	const float CIRCLE_RADIUS = 70.0;

	int i;
	for(i = 0; i < 360; i ++)
	{
		orbitSegments[i] = CIRCLE_RADIUS;
	}
}

void Robot::writeFormationToFile(string filename)
{
	ofstream handle;
	handle.open(filename.c_str(), ios::out | ios::binary);
	if(handle.is_open())
	{
		handle.write((char*)orbitSegments, sizeof(float) * 360);
		handle.close();
	}
	else
	{
		cerr << "Robot::writeFormationToFile() could not open " << filename << endl;
	}
}

void Robot::loadFormationFromFile(string filename)
{
	ifstream handle;
	handle.open(filename.c_str(), ios::in | ios::binary);
	if(handle.is_open())
	{
		handle.read((char*)orbitSegments, sizeof(float) * 360);
		handle.close();
	}
	else
	{
		cerr << "Robot::loadFormationFromFile() could not open " << filename << endl;
	}
}

void Robot::init()
{
	// robot starts out enabled with neither motors active
	enabled = true;
	setLeftMotor(0.0);
	setRightMotor(0.0);

	// reset our orbitting PID error
	oldRightError = 0.0;

	// set some defaults
	crowdedTimer = 0;
	reEvalGroupTimer = RE_EVAL_GROUP_TIME;

	// assign a default circle group; should work no matter which we select (even randomly)
	circleGroup = 0;
}

void Robot::update(int millis)
{
	// disabled robots do not do anything
	if(enabled)
	{
		handleBehaviour(millis);
		handleDifferentialDrive(millis);

		// uncomment to allow robots to become randomly and permanently disabled
		//if(rand() % 50000 == 0)
			//enabled = false;
    }
}

int Robot::getCircleGroup()
{
	return circleGroup;
}

float Robot::getOrbitDistance()
{
	return orbitDistance;
}

float Robot::computeOrbitDistance(int circleGroup, int degrees)
{
	return orbitSegments[(degrees + 358) % 360] +
		   (orbitSegments[(degrees + 358) % 360] * circleGroup);
}

void Robot::handleBehaviour(int millis)
{
	const float FRONT_SENSOR_FOV = M_PI;			// how wide a view the front robot-detector sensor has
	const float FRONT_SENSOR_RANGE = 30.0;			// how far the front sensor can detect a robot

	// robot position and orientation
	vec2 pos = getPos();
	float dir = getDir();

	// used to store results of forward sensor readings
	bool inFront;									// is there an object in front of me?
	float inFrontAngle;								// what is my relative bearing to it?
	float inFrontDistance;							// how far away is it?

	// simple localization sensor (e.g., GPS)
	float bearingToCenter;							// how much do we have to turn to face the world center?
	float rangeToCenter;							// how far away from the world center do we think we are?

	// is a robot in front of us? if so, get the closest one
	inFront = world -> getClosestRobotInFOV(pos, dir, FRONT_SENSOR_RANGE, FRONT_SENSOR_FOV, inFrontAngle, inFrontDistance);

	// use our special sensor to figure out where we are in relation to the center of the world
	world -> getRangeAndBearingToCenter(pos, dir + M_PI_2, &rangeToCenter, &bearingToCenter);

	// our orbiting and slowing behaviour
	handleSteeringAndOrbiting(millis, rangeToCenter, bearingToCenter, inFront, inFrontDistance);

	// our circle group changing behaviour
	handleGroupRadiusChanging(millis, inFront, inFrontDistance, rangeToCenter);
}

void Robot::handleSteeringAndOrbiting(int millis, float rangeToCenter, float bearingToCenter, bool inFront, float inFrontDistance)
{
	const float LEFT_MOTOR_DEFAULT = 0.8;			// default left motor speed
	const float RIGHT_MOTOR_DEFAULT = 0.8;			// default right motor speed

	const float P_GAIN = 5.0;						// proportional gain tuning for our PID (really, PD) controller
	const float D_GAIN = 1.0;						// derivative gain tuning for our PID (really, PD) controller

	const float OBSTACLE_AT_BUMPER_DISTANCE = 30.0;									// how close an object has to be to be considered *too* close
	const float OBSTACLE_CROWDING_SLOWDOWN = OBSTACLE_AT_BUMPER_DISTANCE / 1.2;		// slowdown factor based on distance from frontwards obstacle

	const float LOOK_AHEAD_ANGLE = 0.05;				// compute a point this many radians ahead of us on the circle we're orbiting

	// these motor speeds are adjusted by the PID controller and collision avoidance
	float left = LEFT_MOTOR_DEFAULT;
	float right = RIGHT_MOTOR_DEFAULT;
	float dir = getDir();

	float crowdingSlowdownFactor;

	// compute where we think the orbital center is
	vec2 center = vec2(sinf(bearingToCenter) * rangeToCenter, cosf(bearingToCenter) * rangeToCenter);

	// compute our absolute angle in degrees, from 0 to 360, of our position relative to the formation center
	float angle = (bearingToCenter + dir) + M_PI * 3.0;	// encourage +ve radian angle
	formationDegreeAngle = (int)(glm::degrees(angle)) % 360;

	// our orbital distance is a function of our global angle to the formation center
	orbitDistance = computeOrbitDistance(circleGroup, formationDegreeAngle);

	// now compute a position slightly ahead of where we want to orbit
	float angleToLookAhead = angle - dir - LOOK_AHEAD_ANGLE;
	vec2 lookAhead = center + vec2(sinf(angleToLookAhead) * orbitDistance, cosf(angleToLookAhead) * orbitDistance);

	// compute the direction we need to turn to face the point we're targeting
	float targetOffset = normalizeRadians(M_PI - atan2(lookAhead.y, lookAhead.x));

	// compute error and derivative terms for our PD controller
	float error = targetOffset;
	float derivative = error - oldRightError;

	// assign motor speeds using PD controller
	right -= error * P_GAIN + derivative * D_GAIN;
	left += error * P_GAIN + derivative * D_GAIN;

	// record our old PD error for the next simulation step
	oldRightError = error;

	// slow down proportionally to how close someone is when they're in front of us; this has the
	// really nice property of resulting in a "zipper merge" effect
	if(inFront && inFrontDistance < OBSTACLE_AT_BUMPER_DISTANCE)
	{
		crowdingSlowdownFactor = (inFrontDistance / OBSTACLE_CROWDING_SLOWDOWN);
		left *= crowdingSlowdownFactor;
		right *= crowdingSlowdownFactor;
	}

	// assign our motor speeds to the drive
	setLeftMotor(left);
	setRightMotor(right);
}

void Robot::handleGroupRadiusChanging(int millis, bool inFront, float inFrontDistance, float atRightDistance)
{
	const float CROWDED_DISTANCE = 1.0 + ROBOT_RADIUS_METERS * 2.0;		// if someone is this close to us, we consider changing circle groups

	// see if someone is in front of us
	if(inFront)
	{
		// this part is also important; if someone is directly in front of us (i.e., touching our
		// front bumper or something), and we're moving in roughly the correct circular motion we
		// want, chances are we're doing what we're supposed to (going in a circle) but can't
		// because we're too crowded, so we need to think about changing groups
		if(inFrontDistance < CROWDED_DISTANCE && atRightDistance >= orbitDistance)// && getLeftMotor() >= getRightMotor())
		{
			crowdedTimer += millis;
		}
		else if(inFrontDistance >= CROWDED_DISTANCE)
		{
			// no interference, so just reset our crowded tracking timer
			crowdedTimer = 0;

			// periodically see if we should change circle groups based on our distance from the center
			reEvalGroupTimer -= millis;
			if(reEvalGroupTimer <= 0)
			{
				if(atRightDistance <= computeOrbitDistance(circleGroup + 1, formationDegreeAngle))
				{
					//circleGroup = (atRightDistance - (circleBaseRadius / 2.0f)) / circleBaseRadius;
				}
				reEvalGroupTimer = RE_EVAL_GROUP_TIME;
			}
		}

		// if we've been crowded for too long, decide how we should change groups
		if(crowdedTimer > OVERCROWDED_FED_UP_TIME + (circleGroup * 1000))
		{
			// promote ourselves and reset the group eval timer
			circleGroup ++;
			reEvalGroupTimer = RE_EVAL_GROUP_TIME;

			// reset our timers and make sure we don't make a group change for the next while
			crowdedTimer = 0;
		}
	}
}

void Robot::handleDifferentialDrive(int millis)
{
	const int MILLIS_PER_SECOND = 1000;
	const float TIME_FACTOR = ((float)millis / (float)MILLIS_PER_SECOND);

	const float WHEEL_CENTER_OFFSET = ROBOT_RADIUS_METERS;		// for now we assume the wheels are on the very edge of the robot's base
	const float DIST_BETWEEN_WHEELS = WHEEL_CENTER_OFFSET * 2;

	vec2 pos = getPos();
	float dir = getDir();
	float leftSpeed = leftMotor;
	float rightSpeed = rightMotor;

	// avoid division by zero (ugly edge case) and factor into account the max motor speed
	if(fabs(leftSpeed - rightSpeed) < 0.001)
	{
		if(leftSpeed > 0.0)
			leftSpeed -= 0.001;
		else
			leftSpeed += 0.001;
	}

	leftSpeed *= ROBOT_MOTOR_SPEED;
	rightSpeed *= ROBOT_MOTOR_SPEED;

	// avoid division by zero (ugly edge case) and factor into account the max motor speed
	if(fabs(leftSpeed - rightSpeed) < 0.001)
	{
		if(leftSpeed > 0.0)
			leftSpeed -= 0.001;
		else
			leftSpeed += 0.001;
	}

	// record our old position before we move
	setOldPos(pos);

	// compute our new position based on the kinematics model of a differentially-driven vehicle
	pos.x += ((DIST_BETWEEN_WHEELS * (rightSpeed + leftSpeed)) / (2 * (rightSpeed - leftSpeed))) *
			 (sin(TIME_FACTOR * (rightSpeed - leftSpeed) / DIST_BETWEEN_WHEELS + dir) - sin(dir));
	pos.y += ((DIST_BETWEEN_WHEELS * (rightSpeed + leftSpeed)) / (2 * (rightSpeed - leftSpeed))) *
			 (cos(TIME_FACTOR * (rightSpeed - leftSpeed) / DIST_BETWEEN_WHEELS + dir) - cos(dir));

	// and update our direction, too
	dir += (TIME_FACTOR * (rightSpeed - leftSpeed)) / DIST_BETWEEN_WHEELS;

	// update base object's position and orientation
	setPos(pos);
	setDir(dir);
}

void Robot::render()
{
	vec2 pos = getPos();
	float dir = getDir();

	// visualize the robot
	if(enabled)
	{
		slSetForeColor(circleGroup & 0x04, circleGroup & 0x02, circleGroup & 0x01, 0.4);
		slCircleFill(pos.x, pos.y, ROBOT_RADIUS_METERS, 8);
	}
	else
	{
		slSetForeColor(0.0, 0.0, 0.0, 0.6);
		slCircleFill(pos.x, pos.y, ROBOT_RADIUS_METERS, 8);
	}

	// outline of the robot can just be black
	slSetForeColor(0.0, 0.0, 0.0, 1.0);
	slCircleOutline(pos.x, pos.y, ROBOT_RADIUS_METERS, 8);

	// visualize the direction of the robot
	slLine(pos.x,
		   pos.y,
		   pos.x - cos(dir) * ROBOT_RADIUS_METERS,
		   pos.y + sin(dir) * ROBOT_RADIUS_METERS);
}

void Robot::setLeftMotor(float leftMotor)
{
	this -> leftMotor = clamp(-leftMotor, -1.0f, 1.0f);
}

float Robot::getLeftMotor()
{
	return leftMotor;
}

void Robot::setRightMotor(float rightMotor)
{
	this -> rightMotor = clamp(-rightMotor, -1.0f, 1.0f);
}

float Robot::getRightMotor()
{
	return rightMotor;
}
