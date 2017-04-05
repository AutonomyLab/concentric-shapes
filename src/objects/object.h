#pragma once

#include "glm/glm.hpp"

class Object
{
private:
	glm::vec2 pos;					// global position of object
	glm::vec2 oldPos;				// previous position of object
	float dir;						// direction object is facing
	float mass;						// how much the object weighs
	bool enabled;					// is the object considered active?

	void init();					// initializes to a known state

public:
	Object();
	Object(glm::vec2 pos, float orientation);
	virtual ~Object() = 0;

	// position of object
	virtual glm::vec2 getPos();
	virtual void setPos(glm::vec2 pos);

	// old position of object
	virtual glm::vec2 getOldPos();
	virtual void setOldPos(glm::vec2 oldPos);

	// orientation of object
	virtual float getDir();
	virtual void setDir(float dir);

	// better to move an object with move() than to use setPos()
	virtual void move(glm::vec2 amount);

	// mass of object
	virtual float getMass();
	virtual void setMass(float mass);

	// objects that are disabled cannot collide with other objects
	virtual bool getEnabled();
	virtual void setEnabled(bool enabled);

	// main updating and rendering methods; called every frame
	virtual void update(int millis);
	virtual void render();
};
