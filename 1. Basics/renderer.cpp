#include "precomp.h"

// -----------------------------------------------------------
// Initialize the renderer
// -----------------------------------------------------------

void Renderer::Init()
{
	// create fp32 rgb pixel buffer to render to
	accumulator = (float4*)MALLOC64( SCRWIDTH * SCRHEIGHT * 16 );
	memset( accumulator, 0, SCRWIDTH * SCRHEIGHT * 16 );
	camera = new Camera(scene.GetCameraPos(0), scene.GetCameraTarget(0));
}

float3 Renderer::Trace(Ray& ray)
{
	scene.FindNearest(ray);
	if (ray.objIdx == -1) return 0; // or a fancy sky color
	float3 I = ray.O + ray.t * ray.D;
	float3 N = scene.GetNormal(ray.objIdx, I, ray.D);

	uint seed = 0;
	uint quadIdx = scene.GetRandomLight(seed);
	Quad quad = scene.GetLightQuad(quadIdx);
	float3 L = scene.RandomPointOnLightQuad(quadIdx, seed) - I;
	float dist = length(L);
	L /= dist;
	float cos_o = dot(-L, quad.GetNormal(I));
	float cos_i = dot(L, N);
	if ((cos_o <= 0) || (cos_i <= 0)) return float3(0);

	Ray shadowRay = Ray(I + DBL_EPSILON * L, L, dist - 2 * DBL_EPSILON);
	scene.FindNearest(shadowRay);
	if (shadowRay.objIdx == -1) return float3(0);


	float3 albedo = scene.GetAlbedo(ray.objIdx, I);
	float3 BRDF = albedo / PI;
	float solidAngle = (scene.GetLightArea() * cos_o) / (dist * dist);
	/* visualize normal */ // return (N + 1) * 0.5f;
	/* visualize distance */ // return 0.1f * float3( ray.t, ray.t, ray.t );
	/* visualize albedo */
	return BRDF * scene.GetLightCount() * scene.GetLightColor() * solidAngle * cos_i;
}

// -----------------------------------------------------------
// Main application tick function - Executed once per frame
// -----------------------------------------------------------
void Renderer::Tick( float deltaTime )
{
	// animation
	if (animating) scene.SetTime( anim_time += deltaTime * 0.002f );
	// pixel loop
	Timer t;
	int blacks = 0;

	// lines are executed as OpenMP parallel tasks (disabled in DEBUG)
#pragma omp parallel for schedule(dynamic)
	for (int y = 0; y < SCRHEIGHT; y++)
	{
		// trace a primary ray for each pixel on the line
		for (int x = 0; x < SCRWIDTH; x++)
		{
			float4 pixel = float4(Trace(camera->GetPrimaryRay((float)x, (float)y)), 0);
			// translate accumulator contents to rgb32 pixels
			if (pixel.x == 0 && pixel.y == 0 && pixel.z == 0) {
				blacks++;
			}
			else {
				//cout << pixel.x << " " << pixel.y << " " << pixel.z << "\n";
			}
			screen->pixels[x + y * SCRWIDTH] = RGBF32_to_RGB8( &pixel );
			accumulator[x + y * SCRWIDTH] = pixel;
		}
	}
	cout << "blacks = " << blacks / (640*1024.0) << "\n";

	// performance report - running average - ms, MRays/s
	static float avg = 10, alpha = 1;
	avg = (1 - alpha) * avg + alpha * t.elapsed() * 1000;
	if (alpha > 0.05f) alpha *= 0.5f;
	float fps = 1000.0f / avg, rps = (SCRWIDTH * SCRHEIGHT) / avg;
	printf( "%5.2fms (%.1ffps) - %.1fMrays/s\n", avg, fps, rps / 1000 );
	// handle user input
	camera->HandleInput( deltaTime );
}

// -----------------------------------------------------------
// Update user interface (imgui)
// -----------------------------------------------------------
void Renderer::UI()
{
	// animation toggle
	ImGui::Checkbox( "Animate scene", &animating );
	ImGui::Checkbox("Acceleration structure", &scene.useBVH);
	// ray query on mouse
	Ray r = camera->GetPrimaryRay( (float)mousePos.x, (float)mousePos.y );
	//scene.FindNearest( r );
	//ImGui::Text( "Object id: %i", r.objIdx );
}