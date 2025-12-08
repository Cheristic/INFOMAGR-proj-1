#pragma once
#include "precomp.h"

namespace Tmpl8
{

class Renderer : public TheApp
{
public:
	// game flow methods
	void Init();
	float3 Trace(Ray& ray);
	float3 GetColor( Ray& ray );
	void Tick( float deltaTime );
	void UI();
	void Shutdown() { /* implement if you want to do things on shutdown */ }
	// input handling
	void MouseUp( int button ) { /* implement if you want to detect mouse button presses */ }
	void MouseDown( int button ) { /* implement if you want to detect mouse button presses */ }
	void MouseMove( int x, int y ) { mousePos.x = x, mousePos.y = y; }
	void MouseWheel( float y ) { /* implement if you want to handle the mouse wheel */ }
	void KeyUp( int key ) { /* implement if you want to handle keys */ }
	void KeyDown( int key ) { /* implement if you want to handle keys */ }
	// data members
	int2 mousePos;
	float4* accumulator;
	Scene scene;
	Camera* camera;
	bool animating = true;
	float anim_time = 0;
	bool heatMap;
	bool intersectionHeatMap;
	// color functions
	float remap(float x, float inMin, float inMax, float outMin, float outMax);
	float3 remapToGreenRed();
	uint createRGB(int r, int g, int b);
	int traversalSteps;
	int minTraverses = 10000000;
	int intersectionTests;
	int minIntersects = 10000000;
	int totalPixelsChecked;

	int traversalStepsPrimary;
	int intersectionTestsPrimary;
	int traversalStepsShadow;
	int intersectionTestsShadow;
	uint frames = 0;

};

} // namespace Tmpl8