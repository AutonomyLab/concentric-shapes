#include "objects/object.h"

#include "glm/glm.hpp"
using namespace glm;

Object::Object()
{
	init();
}

Object::Object(vec2 pos, float orientation)
{
	init();
	setPos(pos);
	setDir(orientation);
}

Object::~Object() { }

void Object::init()
{
	dir = 0.0;
	mass = 1.0;
	enabled = true;
}

vec2 Object::getPos()
{
	return pos;
}

void Object::setPos(vec2 pos)
{
	this -> pos = pos;
}

vec2 Object::getOldPos()
{
	return oldPos;
}

void Object::setOldPos(vec2 oldPos)
{
	this -> oldPos = oldPos;
}

float Object::getDir()
{
	return dir;
}

void Object::setDir(float dir)
{
	while( dir < -M_PI ) dir += 2.0*M_PI;
    while( dir >  M_PI ) dir -= 2.0*M_PI;
	this -> dir = dir;

}

void Object::move(vec2 amount)
{
	pos += amount;
}

float Object::getMass()
{
	return mass;
}

void Object::setMass(float mass)
{
	this -> mass = mass;
}

bool Object::getEnabled()
{
	return enabled;
}

void Object::setEnabled(bool enabled)
{
	this -> enabled = enabled;
}

void Object::update(int) { }

void Object::render() { }
