#pragma once

#include "glm/glm.hpp"

#include <vector>

class Object;
class Robot;
class Food;
class Base;

class World
{
private:
	glm::vec2 worldSize;			// size of the world in meters
	glm::vec2 worldHalfSize;		// half-size of the world in meters
	glm::vec2 windowSize;			// size of the window on which to display the world

	int backgroundTexture;			// SIGIL texture ID for the backdrop

	int millisPerStep;				// every call to update() advances the world by this many milliseconds
	unsigned long time;				// current time in milliseconds

	std::vector<Object*> objects;	// all of the objects contained in the world (robots, food, bases, NOT markers), grouped for rendering
	std::vector<Robot*> robots;		// all of the robots contained in the world

	Robot ****robotGrid;			// our spatial partitioning cell contents; 3D array of robot pointers
	int **robotCount;				// the number of robots currently in each spatial partitioning cell; 2D array of ints
	int partitionResolution;		// square unit size of each spatial partitioning cell
	glm::vec2 partitionSize;		// number of cells wide and high our entire spatial partitioning grid is

	glm::vec2 oldMousePos;			// this is used to handle dragging of items in the simulator

	float convergenceFactor;		// [0.0-1.0], representing the degree to which the robots have converged to their correct positions
	float averageOrbitDistance;		// represents the average distance of the robots' distances from their correct positions
	float varianceOrbitDistance;	// variance of the average orbit distance
	float stdDevOrbitDistance;		// standard deviation of average orbit distance

	void updateRobots();			// handle robot processing

	void updateFormationShape();	// real-time modification of the formation shape by clicking and dragging mouse

	// handle some simple physics interactions among the objects in the world
	void handleRobotCollisions(Robot *robot, glm::vec2 robotPos);
	void handleWallCollisions(Robot *robot, glm::vec2 robotPos);

	// rendering for all components of the world
	void renderBackground();
	void renderFormationShape();
	void renderObjects();
	void renderStats();

	// some initialization methods
	void initSpatialPartitioning();

public:
	World(glm::vec2 worldSize, glm::vec2 windowSize, int millisPerStep, int backgroundTexture);
	~World();

	// used by robots to generate a repulsive force away from the walls of the world
	glm::vec2 getWorldSize();

	// add objects to our world at our discretion
	void addRobot(glm::vec2 pos, float dir);

	// robot sensor handling
	void getRangeAndBearingToCenter(glm::vec2 pos, float dir, float *rangeToCenter, float *bearingToCenter);
	bool getClosestRobotInFOV(glm::vec2 pos, float dir, float range, float fov, float &angleTo, float &distanceTo);

	void update();					// advances the simulation by millisPerStep milliseconds
	void render();					// renders the current state of the world

	// get current world time in milliseconds
	long int getTime();
	float getConvergenceFactor();
	float getAverageOrbitDistance();
	float getVarianceOrbitDistance();
	float getStdDevOrbitDistance();
};
