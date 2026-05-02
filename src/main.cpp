#include <raylib.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <algorithm>

// ── Constants ────────────────────────────────────────────────────────────────
// Use #define for compile-time constants that affect array sizes
#define SCREEN_WIDTH  960
#define SCREEN_HEIGHT 540

static int ScreenWidth  = SCREEN_WIDTH;
static int ScreenHeight = SCREEN_HEIGHT;

// ── Camera ───────────────────────────────────────────────────────────────────
static Camera2D camera;

// ── Pan limits (world-space) ─────────────────────────────────────────────────
static float panLimitLeft   = -ScreenWidth * 2.0f;
static float panLimitRight  =  ScreenWidth * 2.0f;
static float panLimitTop    = -ScreenHeight;
static float panLimitBottom =  ScreenHeight;

void InitCamera()
{
    camera.target = Vector2{ (float)SCREEN_WIDTH * 0.5f, (float)SCREEN_HEIGHT * 0.5f };
    camera.offset = Vector2{ ScreenWidth * 0.5f, ScreenHeight * 0.5f };
    camera.rotation = 0.0f;
    camera.zoom = 0.7f;
}

// ── Main ─────────────────────────────────────────────────────────────────────
int main(void)
{
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(ScreenWidth, ScreenHeight, "Raylib C++ 2D Physics Simulation");
    SetTargetFPS(60);

    // Initialize camera centered on the spawn area
    InitCamera();

    // Mouse drag state
    bool isDragging = false;
    Vector2 dragStart = { 0, 0 };
    Vector2 camTargetAtDragStart = { 0, 0 };

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        if (dt > 0.033f) dt = 0.033f;

        // Handle window resize: just update the viewport size and camera offset
        // so the same world point stays centered. Zoom is NOT changed — we just
        // get more (or fewer) pixels to render into.
        if (IsWindowResized())
        {
            ScreenWidth  = GetScreenWidth();
            ScreenHeight = GetScreenHeight();
            
            // Keep camera centered on the same world point
            camera.offset.x = ScreenWidth * 0.5f;
            camera.offset.y = ScreenHeight * 0.5f;
        }

        // ── Camera controls ────────────────────────────────────────────────
        // WASD / Arrow keys panning
        float panSpeed = 300.0f / camera.zoom;
        if (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP))
            camera.target.y -= panSpeed * dt;
        if (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN))
            camera.target.y += panSpeed * dt;
        if (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT))
            camera.target.x -= panSpeed * dt;
        if (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT))
            camera.target.x += panSpeed * dt;

        // Mouse wheel zoom
        float wheel = GetMouseWheelMove();
        if (wheel != 0)
        {
            // Zoom towards mouse cursor
            Vector2 mousePos = GetMousePosition();
            Vector2 worldPos = GetScreenToWorld2D(mousePos, camera);

            camera.zoom *= (1.0f + wheel * 0.1f);
            if (camera.zoom < 0.1f) camera.zoom = 0.1f;
            if (camera.zoom > 10.0f) camera.zoom = 10.0f;

            // Adjust target so the world point under the mouse stays fixed
            Vector2 newWorldPos = GetScreenToWorld2D(mousePos, camera);
            camera.target.x += worldPos.x - newWorldPos.x;
            camera.target.y += worldPos.y - newWorldPos.y;
        }

        // Mouse drag panning
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
        {
            isDragging = true;
            dragStart = GetMousePosition();
            camTargetAtDragStart = camera.target;
        }
        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
        {
            isDragging = false;
        }
        if (isDragging && IsMouseButtonDown(MOUSE_BUTTON_LEFT))
        {
            Vector2 mousePos = GetMousePosition();
            Vector2 delta = {
                (dragStart.x - mousePos.x) / camera.zoom,
                (dragStart.y - mousePos.y) / camera.zoom
            };
            camera.target.x = camTargetAtDragStart.x + delta.x;
            camera.target.y = camTargetAtDragStart.y + delta.y;
        }

        // Clamp camera target to pan limits
        if (camera.target.x < panLimitLeft)  camera.target.x = panLimitLeft;
        if (camera.target.x > panLimitRight) camera.target.x = panLimitRight;
        if (camera.target.y < panLimitTop)    camera.target.y = panLimitTop;
        if (camera.target.y > panLimitBottom) camera.target.y = panLimitBottom;

        
        // Render
        BeginDrawing();
        ClearBackground(Color{ 15, 15, 25, 255 });

        // World-space rendering (affected by camera)
        BeginMode2D(camera);
        DrawCircle(ScreenWidth/2, ScreenHeight/2, 100.0f, BLUE);
        EndMode2D();

        // UI (screen-space, not affected by camera)

        EndDrawing();
        
    }

    CloseWindow();

    return 0;
}