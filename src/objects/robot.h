#pragma once

#include "objects/object.h"

#include "glm/glm.hpp"

#include <string>

class World;

class Robot : public Object
{
private:
	static const int GROUP_CHANGE_DEAD_TIME;			// minimum amount of time we must wait before changing groups again (msec)
	static const int OVERCROWDED_FED_UP_TIME;			// how long after being overcrowded (after the group change time is elapsed)
														// we will change circle groups (msec)
	static const int RE_EVAL_GROUP_TIME;				// how often we change our group radius solely based on distance from center

	static float orbitSegments[360];					// with our orbit divided into segments, at what distance we should be for each segment;
														// this is the core of the formation shape algorithm

	World *world;										// allows us to use sensors to interact with the world

	bool enabled;										// if false, robot does not do anything
	float leftMotor;									// left motor speed, ranging from [-1.0, 1.0]
	float rightMotor;									// right motor speed, ranging from [-1.0, 1.0]

	float oldRightError;								// used for our orbiting PID controller derivative term

	int crowdedTimer;									// tracks how long the robot considers itself to be crowded
	int reEvalGroupTimer;								// controls how often we evaluate our circle group based on distance to center target

	int formationDegreeAngle;							// global angle to formation center
	float orbitDistance;								// robot's currently targeted orbital distance
	int circleGroup;									// currently circle we consider ourselves belonging to (1 is the first inner circle)

	void init();										// sets variables to known states when object is initially constructed

	float computeOrbitDistance(int circleGroup, int degrees);

	// behaviour is broken down into 2 sub-behaviours:
	// steering/orbiting/slowing down; and circle group changing
	void handleBehaviour(int millis);
	void handleSteeringAndOrbiting(int millis, float atRightDistance, float atRightAngle, bool inFront, float inFrontDistance);
	void handleGroupRadiusChanging(int millis, bool inFront, float inFrontDistance, float atRightDistance);

	// lower-level motion control
	void handleDifferentialDrive(int millis);

	// motor control
	void setLeftMotor(float leftMotor);
	float getLeftMotor();
	void setRightMotor(float rightMotor);
	float getRightMotor();

public:
	static const float ROBOT_RADIUS_METERS;						// robot radius in meters (also the wheel radius)
	static const float ROBOT_MOTOR_SPEED;						// robot speed in meters per second

	static const float ROBOT_COLLISION_BOUNDS;					// how close we can get to a robot without colliding against it
	static const float ROBOT_COLLISION_BOUNDS_SQUARED;			// same value above, squared, for faster distance checks

	// real-time formation shape control
	static void updateFormationShape(int angle, float distance);
	static float getFormationShape(int angle);
	static void setupDefaultFormation();						// intializes a simple circular formation

	// saving or loading a formation to a file
	static void writeFormationToFile(std::string filename);
	static void loadFormationFromFile(std::string filename);

	Robot(World *world, glm::vec2 pos, float dir);
	~Robot();

	int getCircleGroup();										// what is our layer ID?
	float getOrbitDistance();									// what orbital distance are we targeting *right now*?

	// basic object update and rendering methods; called every frame
	void update(int millis);
	void render();
};
