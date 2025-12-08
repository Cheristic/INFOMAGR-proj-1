#include <string>
#include "precomp.h"

float Renderer::remap(float x, float inMin, float inMax, float outMin, float outMax)
{
	x = min(max(x, inMin), inMax);
	float t = (x - inMin) / (inMax - inMin);
	return outMin + (outMax - outMin) * t;
}

float3 Renderer::remapToGreenRed()
{
	float3 red(1, 0, 0), green(0, 0.8, 0);

	// Realistically, one ray isn't intersecting more than 80 objects (checked outputs)
	float maxIntersections = (intersectionHeatMap) ? scene.maxIntersectionTests : scene.maxTraversalSteps; // sizeof(BVHNode)* N * 2 + 1;
	maxIntersections = min(maxIntersections, 200.0f);
	maxIntersections = sqrtf(maxIntersections);
	float minIntersections = 0.0f;
	float value = (intersectionHeatMap) ? scene.intersectionTests : scene.traversalSteps;
	value = sqrtf(value);
	float a = remap(value, minIntersections, maxIntersections, 0.0f, 1.0f);

	float3 color = red * a + green * (1.0f - a);
	return color * 255;
}


uint Renderer::createRGB(int r, int g, int b)
{
	return ((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff);
}


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
	int currIntTests = scene.intersectionTests;
	int currTravSteps = scene.traversalSteps;
	intersectionTestsPrimary += currIntTests;
	traversalStepsPrimary += currTravSteps;

	if (ray.objIdx == -1) return 0; // or a fancy sky color
	float3 I = ray.O + ray.t * ray.D;
	float3 N = scene.GetNormal(ray.objIdx, I, ray.D);
	//return scene.GetAlbedo(ray.objIdx, I);


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

	intersectionTestsShadow += scene.intersectionTests - currIntTests;
	traversalStepsShadow += scene.traversalSteps - currTravSteps;
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

	// lines are executed as OpenMP parallel tasks (disabled in DEBUG)
#pragma omp parallel for schedule(dynamic)
	for (int y = 0; y < SCRHEIGHT; y++)
	{
		// trace a primary ray for each pixel on the line
		for (int x = 0; x < SCRWIDTH; x++)
		{
			float4 pixel = float4(Trace(camera->GetPrimaryRay((float)x, (float)y)), 0);
			// translate accumulator contents to rgb32 pixels
			if (!heatMap)
			{
				screen->pixels[x + y * SCRWIDTH] = RGBF32_to_RGB8(&pixel);
			}
			else
			{
				float3 finalColor = remapToGreenRed();
				float4 v(finalColor.x, finalColor.y, finalColor.z, 1.0f);
				screen->pixels[x + y * SCRWIDTH] = createRGB(finalColor.x, finalColor.y, finalColor.z);
			}
			scene.maxIntersectionTests = max(scene.maxIntersectionTests, scene.intersectionTests);
			scene.maxTraversalSteps = max(scene.maxTraversalSteps, scene.traversalSteps);

			if (frames < 100) 
			{
				minIntersects = min(minIntersects, scene.intersectionTests);
				minTraverses = min(minTraverses, scene.traversalSteps);
				traversalSteps += scene.traversalSteps;
				intersectionTests += scene.intersectionTests;
				totalPixelsChecked++;
			}

			scene.intersectionTests = 0;
			scene.traversalSteps = 0;
		}
	}

	// performance report - running average - ms, MRays/s
	static float avg = 10, alpha = 1;
	avg = (1 - alpha) * avg + alpha * t.elapsed() * 1000;
	if (alpha > 0.05f) alpha *= 0.5f;
	float fps = 1000.0f / avg, rps = (SCRWIDTH * SCRHEIGHT) / avg;
	//printf( "%5.2fms (%.1ffps) - %.1fMrays/s\n", avg, fps, rps / 1000 );
	// handle user input
	camera->HandleInput( deltaTime );
	frames++;
	if (frames == 100) 
	{
		cout << "INTERS " << scene.maxIntersectionTests << " " << minIntersects << " " << intersectionTests / totalPixelsChecked << "\n";
		cout << "TRAVERS " << scene.maxTraversalSteps << " " << minTraverses << " " << traversalSteps / totalPixelsChecked << "\n";
		cout << "PRIMARY RAYS: INTERS " << intersectionTestsPrimary / totalPixelsChecked << " TRAVERS " << traversalStepsPrimary / totalPixelsChecked << "\n";
		cout << "SHADOW RAYS: INTERS " << intersectionTestsShadow / totalPixelsChecked << " TRAVERS " << traversalStepsShadow / totalPixelsChecked << "\n";

	}
	//cout << camera->camPos.x << " " << camera->camPos.y << " " << camera->camPos.z << " " << camera->camTarget.x << " " << camera->camTarget.y << " " << camera->camTarget.z << "\n";
}

// -----------------------------------------------------------
// Update user interface (imgui)
// -----------------------------------------------------------
void Renderer::UI()
{
	// animation toggle
	ImGui::Checkbox( "Animate scene", &animating );
	ImGui::Checkbox("Acceleration structure", &scene.accelStruct);
	ImGui::Checkbox("Heat Map", &heatMap); ImGui::SameLine();
	const char* items[] = { "Intersection Tests", "Traversal Steps"};
	static int item_selected_idx = 0; // Here we store our selection data as an index.

	intersectionHeatMap = !item_selected_idx;

	// Pass in the preview value visible before opening the combo (it could technically be different contents or not pulled from items[])
	const char* combo_preview_value = items[item_selected_idx];
	if (ImGui::BeginCombo("", combo_preview_value))
	{
		for (int n = 0; n < IM_ARRAYSIZE(items); n++)
		{
			const bool is_selected = (item_selected_idx == n);
			if (ImGui::Selectable(items[n], is_selected))
				item_selected_idx = n;

			// Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
			if (is_selected)
				ImGui::SetItemDefaultFocus();
		}
		ImGui::EndCombo();
	}

	static int e = scene.accelStructType;
	ImGui::RadioButton("BVH", &e, 0); ImGui::SameLine();
	ImGui::RadioButton("kD-tree", &e, 1); ImGui::SameLine();
	ImGui::RadioButton("Octree", &e, 2);

	scene.accelStructType = e;

	//static int f = scene.SceneIdx;
	//static int fOld = f;
	//ImGui::RadioButton("Scene 1", &f, 0); ImGui::SameLine();
	//ImGui::RadioButton("Scene 2", &f, 1);

	static int g = 0;
	static int gOld = 0;
	ImGui::RadioButton("CamPos 1", &g, 0); ImGui::SameLine();
	ImGui::RadioButton("CamPos 2", &g, 1); ImGui::SameLine();
	ImGui::RadioButton("CamPos 3", &g, 2);

	//scene.SceneIdx = f;
	if (gOld != g)
	{
		cout << "Swapping Positions, remeasuring stats...\n";
		camera->camPos = scene.GetCameraPos(g);
		camera->camTarget = scene.GetCameraTarget(g);
		camera->Update();
		scene.maxIntersectionTests = 0;
		scene.maxTraversalSteps = 0;
		traversalSteps = 0;
		minTraverses = 10000000;
		intersectionTests = 0;
		minIntersects = 10000000;
		totalPixelsChecked = 0;
		frames = 0;

		traversalStepsPrimary = 0;
		intersectionTestsPrimary = 0;
		traversalStepsShadow = 0;
		intersectionTestsShadow = 0;
	}
	gOld = g;

	ImGui::LabelText(std::to_string(scene.maxIntersectionTests).c_str(), "Max # intersection tests:");
	ImGui::LabelText(std::to_string(scene.maxTraversalSteps).c_str(), "Max # Traversal steps:");
	
	// ray query on mouse
	Ray r = camera->GetPrimaryRay( (float)mousePos.x, (float)mousePos.y );
	//scene.FindNearest( r );
	//ImGui::Text( "Object id: %i", r.objIdx );
}