@echo off

:: define some useful vars
set circles=circles-release.exe
set sim_time=120000
set repetitions=10
set formations=circle rectangle triangle cross

:: number of robots we want to test with
set min_robots=50
set max_robots=500
set robot_inc=50

:: print intro message
echo ------------------------------
echo running circles experiments...

:: iterate over our formations
for %%f in (%formations%) do (

	echo ----RUNNING FORMATION %%f----

	:: vary the number of robots
	for /l %%r in (%min_robots%, %robot_inc%, %max_robots%) do (

		:: and now our repetitions
		for /l %%s in (1, 1, %repetitions%) do (
			::echo %circles% 0 %%r %sim_time% formations/%%f.form stats/%%f_r%%r.csv
			%circles% 0 %%r %sim_time% formations/%%f.form stats/%%f_r%%r.csv
		)
	)
)

:: and...we're done!
echo ----------------------------
echo circles experiments complete
echo ----------------------------
exit /B %ERRORLEVEL%
