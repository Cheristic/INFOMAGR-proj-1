#include "precomp.h"

// -----------------------------------------------------------
// Initialize the renderer
// -----------------------------------------------------------


void Renderer::Init()
{
	// create fp32 rgb pixel buffer to render to
	accumulator = (float4*)MALLOC64( SCRWIDTH * SCRHEIGHT * 16 );
	memset( accumulator, 0, SCRWIDTH * SCRHEIGHT * 16 );
	bvh = BVH();
	kdtree = KDTree();
}

// -----------------------------------------------------------
// Main application tick function - Executed once per frame
// -----------------------------------------------------------
void Renderer::Tick( float deltaTime )
{
	// animation
	if (animating) scene.SetTime( anim_time += deltaTime * 0.002f );
	// pixel loop
	Ray ray;
	Timer t;
	// lines are executed as OpenMP parallel tasks (disabled in DEBUG)
#pragma omp parallel for schedule(dynamic)
	for (int y = 0; y < SCRHEIGHT; y++)
	{
		// trace a primary ray for each pixel on the line
		for (int x = 0; x < SCRWIDTH; x++)
		{
			ray = camera.GetPrimaryRay((float)x, (float)y);
			float4 pixel;
			if (useBVH) {
				pixel = float4(bvh.Trace(ray, bvh.rootNodeIdx), 0);
			}
			else {
				pixel = float4(kdtree.Trace(ray, kdtree.rootNodeIdx), 0);
			}
			 
			// translate accumulator contents to rgb32 pixels
			screen->pixels[x + y * SCRWIDTH] = RGBF32_to_RGB8( &pixel );
			accumulator[x + y * SCRWIDTH] = pixel;
		}
	}
	// performance report - running average - ms, MRays/s
	static float avg = 10, alpha = 1;
	avg = (1 - alpha) * avg + alpha * t.elapsed() * 1000;
	if (alpha > 0.05f) alpha *= 0.5f;
	float fps = 1000.0f / avg, rps = (SCRWIDTH * SCRHEIGHT) / avg;
	printf( "%5.2fms (%.1ffps) - %.1fMrays/s\n", avg, fps, rps / 1000 );
	// handle user input
	camera.HandleInput( deltaTime );
}

// -----------------------------------------------------------
// Update user interface (imgui)
// -----------------------------------------------------------
void Renderer::UI()
{
	// animation toggle
	ImGui::Checkbox( "Animate scene", &animating );
	// ray query on mouse
	Ray r = camera.GetPrimaryRay( (float)mousePos.x, (float)mousePos.y );
	//scene.FindNearest( r );
	//ImGui::Text( "Object id: %i", r.objIdx );
}