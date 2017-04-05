# Self-Organization of a Robot Swarm into Concentric Shapes

Contains the source code, scripts, and results for "Self-Organization of a Robot Swarm into Concentric Shapes".

This is a Code::Blocks project. (If you do not have the excellent Code::Blocks IDE, you can get it [here](http://codeblocks.org/downloads/26).) This project should compile and run out-of-the-box, since all required libraries are included.

The underlying media library for this project is SIGIL, which is a simple, cross-platform, open-source C library for sound, input, and basic graphics. The website for SIGIL is [here](http://www.libsigil.com). A local copy of the required Windows SIGIL libraries is already included, so you do not need to download them.

Although this project was developed in Windows, it can be ported to Linux with minimal effort since all required libraries are cross-platform.

## Folders
**formations**: contains binary files describing the formations used in the experiments
**lib**: contains static library (currently, only _libsigil.dll.a_) used by the project
**png**: contains any required graphics (currently, only a tilable checkerboard pattern)
**src**: contains all C++ source code, including a local copy of the fantastic GLM library, which can be downloaded [here](http://glm.g-truc.net/0.9.8/index.html)
**stats**: results of experiments; final results are aggregated in _.xlsx_ files
**ttf**: font files used by the project for graphical debugging output

## Contact

If you have any questions about this project or software, please feel free to contact Geoff Nagy at gnagy@sfu.ca.
