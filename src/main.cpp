#include "sl/sl.h"

#include "world/world.h"
#include "objects/robot.h"

#include "glm/glm.hpp"
#include "glm/gtc/random.hpp"
using namespace glm;

#include <cstdlib>
#include <ctime>
#include <cstring>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <fenv.h>
using namespace std;

/*
args:
	circles [gui] [num_robots] [max_time_secs] [formation_file] [results_file]
*/

int main(int args, char *argv[])
{
	const int SAVE_STATS_INTERVAL_MS = 1000;				// save stats this often (in milliseconds)
	const int DEFAULT_NUM_ROBOTS = 400;						// if not args specified, use this number of robots
	const int WINDOW_WIDTH = 1200;							// width of the window in pixels (or cm)
	const int WINDOW_HEIGHT = 1200;							// height of the window in pixels (or cm)
	const char *WINDOW_TITLE = "Circles";					// name displayed in window title bar
	const int MILLIS_PER_TIME_STEP = 20;					// how many milliseconds to simulate per update step

	const float STARTING_MARGIN = 0.8;						// 0.8 equates to inner 60%
	const vec2 ROBOT_MIN = vec2(WINDOW_WIDTH * STARTING_MARGIN, WINDOW_HEIGHT * STARTING_MARGIN);
	const vec2 ROBOT_MAX = vec2(WINDOW_WIDTH - ROBOT_MIN.x, WINDOW_HEIGHT - ROBOT_MIN.y);

	bool gui = true;										// is the gui active?
	int numRobots = DEFAULT_NUM_ROBOTS;						// how many robots do we want?
	int maxTime = -1;										// how long should the simulation run for?
	string formationFilename = "formations/circle.form";	// filename of the formation we want to load
	ofstream statsFile;										// file handle for stats reporting

	World *world;											// handle to all-encompassing world simulator
	int backgroundTexture = -1;								// tiled checkboard texture, if in GUI mode
	bool done = false;										// should the main loop end?
	bool paused = false;									// is the simulation paused?
	bool pauseKey = false;									// is the pause key down?
	int i;

	// randomize everything
	srand(time(NULL));

	// super simple; if we have enough args, use them; otherwise, we stick with the defaults already assigned
	if(args == 6)
	{
		gui = strcmp(argv[1], "1") == 0;					// display a gui? (not a good idea for very large WINDOW_WIDTH/HEIGHT values)
		numRobots = atoi(argv[2]);							// how many robots do we want?
		maxTime = atoi(argv[3]);							// how many seconds do we run the simulation for?
		formationFilename = string(argv[4]);				// what formation file do we want the robots to use?

		// open our stats file
		statsFile.open(argv[5], ios::app);
		if(statsFile.is_open())
		{
			// print a header line---remember, we'll be appending to this file many times (with each trial in this configuration)
			statsFile << "-------------------------------------------------------------------------------------------------" << endl;
			statsFile << "time, convergence factor, orbit distance average, orbit distance variance, orbit distance std dev" << endl;
		}
		else
		{
			cout << "---ERR: circles could not open " << argv[5] << " for appending" << endl;
			exit(1);
		}
	}

	// initialize SIGIL and open our graphical window
	if(gui)
	{
		slWindow(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE, 0);
		slSetBackColor(1.0, 1.0, 1.0);
		slSetFont(slLoadFont("ttf/white_rabbit.ttf"), 16);
		slSetSpriteTiling(16.0, 16.0);
		backgroundTexture = slLoadTexture("png/background.png");
	}

	// create our world; for now, window size matches the world size
	cout << "running with " << (gui ? "" : "no ") << "gui, " << numRobots << " robots";
	if(maxTime == -1)
		cout << ", until user quits manually" << endl;
	else
		cout << ", for " << maxTime << "ms" << endl;
	world = new World(vec2(WINDOW_WIDTH, WINDOW_HEIGHT), vec2(WINDOW_WIDTH, WINDOW_HEIGHT), MILLIS_PER_TIME_STEP, backgroundTexture);

	// define the formation we want to use
	Robot::loadFormationFromFile(formationFilename);
	//Robot::setupDefaultFormation();				// uncomment for simple circle (also in "formations/circle.form")

	// add the specified number of robots
	for(i = 0; i < numRobots; i ++)
	{
		world -> addRobot(linearRand(ROBOT_MIN, ROBOT_MAX), linearRand(-M_PI, M_PI));
	}

	// keep looping until user closes window or presses ESC
	while(!done)
	{
		// proceed with the simulation if we're not paused
		if(!paused)
		{
			// updating step always happens
			world -> update();

			// save stats every SAVE_STATS_INTERVAL_MS milliseconds
			if(world -> getTime() % SAVE_STATS_INTERVAL_MS == 0)
			{
				statsFile << setprecision(9) << world -> getTime() / 1000 << ", ";
				statsFile << setprecision(2) << world -> getConvergenceFactor() << ", ";
				statsFile << setprecision(5) << world -> getAverageOrbitDistance() << ", ";
				statsFile << setprecision(9) << world -> getVarianceOrbitDistance() << ", ";
				statsFile << setprecision(9) << world -> getStdDevOrbitDistance() << endl;
			}
		}

		// rendering only occurs if we're in GUI mode
		if(gui)
		{
			world -> render();
			slRender();
		}

		// control pausing
		if(slGetKey('P'))
		{
			if(!pauseKey)
			{
				pauseKey = true;
				paused = !paused;
			}
		}
		else
		{
			pauseKey = false;
		}

		// is the simulation over? either the user has quit, or we've run out of time
		if(gui)
			done = slShouldClose() || slGetKey(SL_KEY_ESCAPE);
		else
			done = world -> getTime() >= maxTime;
	}

	// close our stats file, if any
	if(statsFile.is_open())
	{
		statsFile << endl;
		statsFile.close();
	}

	// save the formation segment distances to file
	//Robot::writeFormationToFile("formations/new_formation_name_here.form");

	// shut down the world
	delete world;
	cout << ": simulation complete" << endl;

	// we're done; close the window and terminate SIGIL
	if(gui)
	{
		slClose();
	}
	return 0;
}
