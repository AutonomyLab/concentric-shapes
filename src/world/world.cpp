#include "world/world.h"

#include "objects/robot.h"

#include "sl/sl.h"

#include "util/util.h"

#include "glm/glm.hpp"
#include "glm/gtc/random.hpp"
using namespace glm;

#include <vector>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <cstring>
using namespace std;

World::World(vec2 worldSize, vec2 windowSize, int millisPerStep, int backgroundTexture)
{
	this -> worldSize = worldSize;
	this -> windowSize = windowSize;
	this -> millisPerStep = millisPerStep;
	this -> backgroundTexture = backgroundTexture;

	worldHalfSize = worldSize / 2.0f;
	initSpatialPartitioning();

	// reset progress stats
	time = 0;
}

void World::initSpatialPartitioning()
{
	const int SPATIAL_PARTITIONING_RESOLUTION = 30;				// cell partition square size in cm

	// we can only fit so many robots into one partition, due to the robots' sizes
	const int MAX_ROBOTS_PER_PARTITION = 1 + (SPATIAL_PARTITIONING_RESOLUTION * SPATIAL_PARTITIONING_RESOLUTION) / (Robot::ROBOT_RADIUS_METERS * 2);
	int i, j;

	// save partition information for later on in the simulation
	partitionResolution = SPATIAL_PARTITIONING_RESOLUTION;
	partitionSize = ceil(worldSize / vec2(partitionResolution));

	// allocate memory for our robot cells; this is a 3D array of Robot pointers
	robotGrid = new Robot***[(int)partitionSize.y];

	// allocate memory for our robot cell robot count; this is a 2D array of ints
	robotCount = new int*[(int)partitionSize.y];

	// allocate the columns of our spatial partitioning grid
	for(i = 0; i < partitionSize.y; i ++)
	{
		robotGrid[i] = new Robot**[(int)partitionSize.x];
		robotCount[i] = new int[(int)partitionSize.x];

		// now allocate the list of robot pointers for each cell, and set the current robot count to zero for each cell
		for(j = 0; j < partitionSize.x; j ++)
		{
			robotGrid[i][j] = new Robot*[MAX_ROBOTS_PER_PARTITION];
			robotCount[i][j] = 0;
		}
	}
}

World::~World()
{
	vector<Object*>::iterator i = objects.begin();
	Object *toDelete;
	int j, k;

	// delete all objects and robots
	while(i != objects.end())
	{
		toDelete = *i;
		i = objects.erase(i);
		delete toDelete;
	}

	// delete spatial partitioning grid---be vewy vewy careful...
	for(j = 0; j < partitionSize.y; j ++)
	{
		for(k = 0; k < partitionSize.x; k ++)
		{
			delete[] robotGrid[j][k];
		}
		delete[] robotCount[j];
		delete[] robotGrid[j];
	}
	delete[] robotCount;
	delete[] robotGrid;
}

vec2 World::getWorldSize()
{
	return worldSize;
}

long int World::getTime()
{
	return time;
}

float World::getConvergenceFactor()
{
	return convergenceFactor;
}

float World::getAverageOrbitDistance()
{
	return averageOrbitDistance;
}

float World::getVarianceOrbitDistance()
{
	return varianceOrbitDistance;
}

float World::getStdDevOrbitDistance()
{
	return stdDevOrbitDistance;
}

void World::addRobot(vec2 pos, float dir)
{
	Robot *robot = new Robot(this, pos, dir);
	robots.push_back(robot);
	objects.push_back(robot);
}

void World::getRangeAndBearingToCenter(glm::vec2 pos, float dir, float *rangeToCenter, float *bearingToCenter)
{
	vec2 offset = worldHalfSize - pos;
	*rangeToCenter = length(offset);
	*bearingToCenter = normalizeRadians(M_PI - atan2(offset.y, offset.x) - dir);
}

bool World::getClosestRobotInFOV(glm::vec2 pos, float dir, float range, float fov, float &angleTo, float &distanceTo)
{
	const float MAX_RANGE_SQUARED = range * range;
	const float MIN_RANGE = Robot::ROBOT_RADIUS_METERS * 2.0;

	vec2 robotPos;
	float closestDistSquared = FLT_MAX;
	float distSquared;
	bool found = false;									// did we find a robot at all?
	bool canStop = false;								// early exit if the closest robot is right against our bumper

	int robotGridX = pos.x / partitionResolution;		// where the robot is on the grid
	int robotGridY = pos.y / partitionResolution;		// where the robot is on the grid

	int numRobotsInCell;								// number of robots in the current spatial partitioning cell
	Robot *curr;										// current robot in the cell we're processing

	int i, j, k;
	int x, y;

	// perform the distance checks only on the 8 (9, really, since we include our own) neighbouring cells' robots
	for(j = -1; j <= 1 && !canStop; j ++)
	{
		for(i = -1; i <= 1 && !canStop; i ++)
		{
			// figure our where we are in our spatially partitioned grid
			x = robotGridX + i;
			y = robotGridY + j;

			// make sure we're in range of the partition grid
			if(x < partitionSize.x && x >= 0 && y < partitionSize.y && y >= 0)
			{
				// iterate through all of the robots in this current grid cell
				numRobotsInCell = robotCount[y][x];
				for(k = 0; k < numRobotsInCell && !canStop; k ++)
				{
					// get the current robot in our cell list and determine how far away we are from it
					curr = *(&robotGrid[y][x][k]);
					robotPos = curr -> getPos();
					distSquared = (robotPos.x - pos.x) * (robotPos.x - pos.x) +
								  (robotPos.y - pos.y) * (robotPos.y - pos.y);

					// if we're close enough, but far enough to be a different robot, then process it
					if(distSquared < closestDistSquared && distSquared < MAX_RANGE_SQUARED && distSquared > 5.0)
					{
						// compute relative angle to the robot to see if we can see it with our sensor's FOV
						angleTo = M_PI - atan2(robotPos.y - pos.y, robotPos.x - pos.x) - dir;
						angleTo = normalizeRadians(angleTo);
						if(fabs(angleTo) < fov / 2.0)
						{
							// we can see it; record it for later
							closestDistSquared = distSquared;
							distanceTo = sqrt(distSquared);
							found = true;

							// if we found someone right up against us, we won't find anyone else closer; exit
							if(distanceTo <= MIN_RANGE)
							{
								canStop = true;
							}
						}
					}
				}
			}
		}
	}

	return found;
}

void World::updateFormationShape()
{
	const float MOUSE_INC = 0.1;

	float angle;
	float distance;

	vec2 mousePos(slGetMouseX(), slGetMouseY());
	vec2 mouseDir;
	vec2 mouseOffset;
	vec2 mouseInc;
	float mouseTravel;
	float i;

	// only update if the left mouse button is down
	if(slGetMouseButton(SL_MOUSE_BUTTON_LEFT))
	{
		// mouse may have moved more than one pixel, so compute intermediate positions as well
		mouseTravel = length(mousePos - oldMousePos);
		mouseDir = (oldMousePos - mousePos) / mouseTravel;
		for(i = 0.0; i < mouseTravel; i += MOUSE_INC)
		{
			// compute an intermediate position between the current mouse and the old mouse position
			mouseInc = mousePos + (mouseDir * i);
			mouseOffset = worldHalfSize - mouseInc;
			distance = length(mouseOffset);

			// get the angle of the mouse to the center and wrap from 0 to 2PI
			angle = M_PI_2 - atan2(mouseOffset.y, mouseOffset.x);
			angle = normalizeRadians(angle) + M_PI;

			// now convert angle to degrees and update robot distance at that angle
			angle = (angle / M_PI) * 180.0;
			Robot::updateFormationShape((int)angle, distance);
		}
	}

	oldMousePos = mousePos;
}

void World::renderFormationShape()
{
	const int NUM_LAYERS = 7;			// how many layers of the formation to draw

	int i, j;
	float currAngle, prevAngle;
	float currDist, prevDist;
	vec2 currPos, prevPos;

	for(i = 0; i < NUM_LAYERS; i ++)
	{
		slSetForeColor(i & 0x04, i & 0x02, i & 0x01, 1.0);

		for(j = 1; j <= 360; j ++)
		{
			currAngle = (j / 180.0) * M_PI;
			prevAngle = ((j - 1) / 180.0) * M_PI;

			currDist = Robot::getFormationShape(j % 360) * (i + 1);
			prevDist = Robot::getFormationShape((j - 1) % 360) * (i + 1);

			currPos = vec2(sinf(currAngle) * currDist, cosf(currAngle) * currDist);
			prevPos = vec2(sinf(prevAngle) * prevDist, cosf(prevAngle) * prevDist);

			currPos += worldHalfSize;
			prevPos += worldHalfSize;

			slLine(currPos.x, currPos.y, prevPos.x, prevPos.y);
		}
	}
}

void World::updateRobots()
{
	const float ACCEPTABLE_DISTANCE = Robot::ROBOT_RADIUS_METERS * 2.0;
	const vec2 CENTER = worldHalfSize;

    float dist;
    float diff;
    float orbitDistance;
    float orbitDistances[robots.size()];
    int j;

	vector<Robot*>::iterator i;
	Robot *curr;
	vec2 robotPos;
	int y, x;

	// zero out our robot spatial partitioning grid
	for(y = 0; y < partitionSize.y; y ++)
	{
		memset(&robotCount[y][0], 0, sizeof(int) * partitionSize.x);
	}

	// insert robots in their appropriate grid cell for spatial partitioning; this must
	// be done before the robots' update steps, since the robots requires this information
	// from the world (for their sensors)
	for(i = robots.begin(); i != robots.end(); ++i)
	{
		// insert our robot into the spatial partitioning grid
		robotPos = (*i) -> getPos();
		y = robotPos.y / partitionResolution;
		x = robotPos.x / partitionResolution;
		robotGrid[y][x][robotCount[y][x]++] = *i;		// increase # of robots in this cell
	}

	// zero out our stats
	convergenceFactor = 0.0;
	averageOrbitDistance = 0.0;
	varianceOrbitDistance = 0.0;
	stdDevOrbitDistance = 0.0;

	// now we can do the main robot processing
	for(i = robots.begin(), j = 0; i != robots.end(); ++i, ++j)
	{
		// get handle to robot
		curr = *i;

		// handle motion and physics
		curr -> update(millisPerStep);
		robotPos = curr -> getPos();
		handleRobotCollisions(curr, robotPos);
		handleWallCollisions(curr, robotPos);

		// calcuate how much the robots have converged
		//robotPos = curr -> getPos();
		dist = length(CENTER - robotPos);
		orbitDistance = curr -> getOrbitDistance();
		diff = fabs(dist - orbitDistance);
		if(diff < ACCEPTABLE_DISTANCE)
		{
			convergenceFactor += 1.0;
		}

		// accumulate an average distance (in robot radii) of how close the robots are to their correct positions
		orbitDistances[j] = (diff / Robot::ROBOT_RADIUS_METERS);
		averageOrbitDistance += orbitDistances[j];
    }

    // now divide by the number of robots, giving us the percentage of robots in their proper orbits
    if(robots.size() > 0)
    {
		// convergence factor and average orbit distance are easy
		convergenceFactor /= (float)robots.size();
		averageOrbitDistance /= (float)robots.size();

		// compute our variance and standard deviation; we require the mean, so this must be done here
		for(j = 0; j < (int)robots.size(); j ++)
		{
			varianceOrbitDistance += (orbitDistances[j] - averageOrbitDistance) * (orbitDistances[j] - averageOrbitDistance);
		}

		// now finalize our variance and std dev calculations
		varianceOrbitDistance /= (float)robots.size();
		stdDevOrbitDistance = sqrt(varianceOrbitDistance);
	}
}

void World::handleRobotCollisions(Robot *robot, vec2 robotPos)
{
	int robotGridX = robotPos.x / partitionResolution;		// where the robot is on the grid
	int robotGridY = robotPos.y / partitionResolution;		// where the robot is on the grid
	int numRobotsInCell;			// number of robots in the current spatial partitioning cell
	Robot *otherRobot;				// other robot we're checking collision against
	vec2 otherRobotPos;				// position of other robot we're checking collision against
	vec2 collisionOffset;			// vector from other robot to given robot
	float collisionDist;			// how close we are to the other robot
	float collisionDepth;			// how far we've penetrated into the other robot
	float totalMass;				// mass of given robot plus the one we've collided with
	int i, j, k;
	int x, y;

	// perform the checks only on the 8 (9, really, since we include our own) neighbouring cells' robots
	for(j = -1; j <= 1; j ++)
	{
		for(i = -1; i <= 1; i ++)
		{
			// figure our where we are in our spatially partitioned grid
			x = robotGridX + i;
			y = robotGridY + j;

			// make sure we're in range of the partition grid
			if(x < partitionSize.x && x >= 0 && y < partitionSize.y && y >= 0)
			{
				// iterate through all of the robots in this current grid cell
				numRobotsInCell = robotCount[y][x];
				for(k = 0; k < numRobotsInCell; k ++)
				{
					// get the current robot in our cell list and determine how far away we are from it
					otherRobot = *(&robotGrid[y][x][k]);
					otherRobotPos = otherRobot -> getPos();
					collisionOffset = otherRobotPos - robotPos;
					collisionDist = (collisionOffset.x * collisionOffset.x) + (collisionOffset.y * collisionOffset.y);

					// robot cannot collide against itself
					if(collisionDist < Robot::ROBOT_COLLISION_BOUNDS_SQUARED && collisionDist > 1.0)
					{
						totalMass = robot -> getMass() + otherRobot -> getMass();
						collisionDist = sqrt(collisionDist);
						collisionDepth = (collisionDist - Robot::ROBOT_COLLISION_BOUNDS);
						collisionOffset /= collisionDist;
						robot -> setPos(robotPos + collisionOffset * collisionDepth * (otherRobot -> getMass() / totalMass));
						otherRobot -> setPos(otherRobotPos - collisionOffset * collisionDepth * (robot -> getMass() / totalMass));
					}
				}
			}
		}
	}
}

void World::handleWallCollisions(Robot *robot, vec2 robotPos)
{
	// how close to the edges of the world the robots can get
	const float MIN_WORLD_BOUNDS_X = 0.0 + Robot::ROBOT_RADIUS_METERS;
	const float MAX_WORLD_BOUNDS_X = worldSize.x - Robot::ROBOT_RADIUS_METERS;
	const float MIN_WORLD_BOUNDS_Y = 0.0 + Robot::ROBOT_RADIUS_METERS;
	const float MAX_WORLD_BOUNDS_Y = worldSize.y - Robot::ROBOT_RADIUS_METERS;

	// test against walls to make sure we can't step outside the environment
	if(robotPos.x < MIN_WORLD_BOUNDS_X) robotPos.x += (MIN_WORLD_BOUNDS_X - robotPos.x);
	if(robotPos.x > MAX_WORLD_BOUNDS_X) robotPos.x -= (robotPos.x - MAX_WORLD_BOUNDS_X);
	if(robotPos.y < MIN_WORLD_BOUNDS_Y) robotPos.y += (MIN_WORLD_BOUNDS_Y - robotPos.y);
	if(robotPos.y > MAX_WORLD_BOUNDS_Y) robotPos.y -= (robotPos.y - MAX_WORLD_BOUNDS_Y);
	robot -> setPos(robotPos);
}

void World::update()
{
	time += millisPerStep;
	updateFormationShape();
	updateRobots();

	if(time % 1000 == 0)
	{
		cout << "\r[";
		cout << setfill('0') << setw(2) << (time / 1000) / 60 << ":";
		cout << setfill('0') << setw(2) << (time / 1000) % 60;
		cout << "] --- ";
		cout << "CVF: " << setprecision(2) << convergenceFactor << "    ";
		cout << "AOD: " << setprecision(5) << averageOrbitDistance << "    ";
		cout << "SOD: " << setprecision(5) << stdDevOrbitDistance << "    ";
		cout.flush();
	}
}

void World::renderBackground()
{
	const float CROSS_HALF_SIZE = 5.0;

	if(backgroundTexture >= 0)
	{
		slSetForeColor(1.0, 1.0, 1.0, 1.0);
		slSprite(backgroundTexture, worldHalfSize.x, worldHalfSize.y, worldSize.x, worldSize.y);

		slSetForeColor(0.0, 0.0, 0.0, 1.0);
		slLine(worldHalfSize.x - CROSS_HALF_SIZE, worldHalfSize.y, worldHalfSize.x + CROSS_HALF_SIZE, worldHalfSize.y);
		slLine(worldHalfSize.x, worldHalfSize.y - CROSS_HALF_SIZE, worldHalfSize.x, worldHalfSize.y + CROSS_HALF_SIZE);
	}
}

void World::renderObjects()
{
	vector<Object*>::iterator i;
	for(i = objects.begin(); i != objects.end(); ++i)
	{
		(*i) -> render();
	}
}

void World::renderStats()
{
	stringstream ss;

	// basic simulation timing stats
	ss << "time: ";
	ss << setfill('0') << setw(2) << (time / 1000) / 60 << ":";
	ss << setfill('0') << setw(2) << (time / 1000) % 60 << endl;
	ss << "fps: " << (int)(1.0f / slGetDeltaTime()) << endl;
	ss << "cvf: " << setprecision(2) << getConvergenceFactor() << endl;
	ss << "aod: " << setprecision(5) << getAverageOrbitDistance() << endl;

	slSetForeColor(0.0, 0.0, 0.0, 1.0);
	slText(10, worldSize.y - 20, ss.str().c_str());
}

void World::render()
{
	renderBackground();
	//renderFormationShape();
	renderObjects();
	//renderStats();
}
