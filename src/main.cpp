#include <raylib.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ── Constants ────────────────────────────────────────────────────────────────
// World dimensions (the simulation runs in a 1920x1080 coordinate space)
#define WORLD_WIDTH  1920
#define WORLD_HEIGHT 1080

// Initial window dimensions (960x540 = 2x downscale of the world)
#define SCREEN_WIDTH  960
#define SCREEN_HEIGHT 540

static int ScreenWidth  = SCREEN_WIDTH;
static int ScreenHeight = SCREEN_HEIGHT;

// ── Simulation Constants ─────────────────────────────────────────────────────
const int NumSwarms = 3;
const int BoidsPerSwarm = 1500;
const int BoidCount = NumSwarms * BoidsPerSwarm;
const float BoidSize = 16.0f;
const float MaxSpeed = 200.0f;
const float MaxForce = 100.0f;
const float PerceptionRadius = 80.0f;
const float PerceptionRadiusSq = PerceptionRadius * PerceptionRadius;
const float SeparationRadius = 30.0f;
const float SeparationRadiusSq = SeparationRadius * SeparationRadius;

// Boid rule weights
const float SeparationWeight = 1.5f;
const float AlignmentWeight = 1.0f;
const float CohesionWeight = 1.0f;

// Boundary behavior
const float BoundaryMargin = 40.0f;
const float TurnForce = 300.0f;

// Spatial grid constants (based on world dimensions)
const int GridCellSize = (int)PerceptionRadius;
const int GridCols = WORLD_WIDTH / GridCellSize + 2;
const int GridRows = WORLD_HEIGHT / GridCellSize + 2;
const int MaxBoidsPerCell = 64;

// ── Camera ───────────────────────────────────────────────────────────────────
static Camera2D camera;

// ── Pan limits (world-space) ─────────────────────────────────────────────────
static float panLimitLeft   = -WORLD_WIDTH * 2.0f;
static float panLimitRight  =  WORLD_WIDTH * 2.0f;
static float panLimitTop    = -WORLD_HEIGHT;
static float panLimitBottom =  WORLD_HEIGHT;

// ── Boid Data ────────────────────────────────────────────────────────────────
struct Boid
{
    Vector2 Position;
    Vector2 Velocity;
    Vector2 Acceleration;
    int SwarmIndex;
    Color Color;
};

static Boid boids[BoidCount];
static Color swarmColors[NumSwarms];

// Spatial grid: for each cell, store indices of boids in that cell
static int gridCellCounts[GridCols * GridRows];
static int gridCells[GridCols * GridRows * MaxBoidsPerCell];

// Pre-computed triangle vertex offsets (avoid sin/cos per boid)
static const float TipOffset = BoidSize * 0.6f;
static const float BackOffset = BoidSize * 0.3f;
static const float LeftAngle = 2.5f;
static const float RightAngle = -2.5f;
static const float CosLeft = cosf(LeftAngle);
static const float SinLeft = sinf(LeftAngle);
static const float CosRight = cosf(RightAngle);
static const float SinRight = sinf(RightAngle);

// ── Forward Declarations ─────────────────────────────────────────────────────
static void InitBoids();
static void InitSwarmColors();
static int GetCellIndex(float x, float y);
static void BuildSpatialGrid();
static void UpdateBoids(float dt);
static void DrawBoids();
static void DrawBorders();
static float InvSqrt(float x);

// ── Camera ───────────────────────────────────────────────────────────────────
void InitCamera()
{
    camera.target = Vector2{ (float)WORLD_WIDTH * 0.5f, (float)WORLD_HEIGHT * 0.5f };
    camera.offset = Vector2{ ScreenWidth * 0.5f, ScreenHeight * 0.5f };
    camera.rotation = 0.0f;
    // Zoom so the full 1920x1080 world fits in the current window.
    // At 960x540, zoom = 0.5x (world is 2x larger than window).
    camera.zoom = (float)ScreenWidth / WORLD_WIDTH;
}

// ── Helper Functions ─────────────────────────────────────────────────────────
static float InvSqrt(float x)
{
    return 1.0f / sqrtf(x);
}

static int GetCellIndex(float x, float y)
{
    int col = (int)(x / GridCellSize);
    int row = (int)(y / GridCellSize);
    // Clamp to valid range
    if (col < 0) col = 0;
    if (col >= GridCols) col = GridCols - 1;
    if (row < 0) row = 0;
    if (row >= GridRows) row = GridRows - 1;
    return row * GridCols + col;
}

// ── Initialization ───────────────────────────────────────────────────────────
static void InitSwarmColors()
{
    for (int s = 0; s < NumSwarms; s++)
    {
        float hue = (float)s / NumSwarms * 360.0f;
        swarmColors[s] = ColorFromHSV(hue, 0.85f, 0.95f);
    }
}

static void InitBoids()
{
    float margin = 80.0f;
    int index = 0;

    for (int s = 0; s < NumSwarms; s++)
    {
        float regionWidth = (WORLD_WIDTH - 2 * margin) / NumSwarms;
        float regionStartX = margin + s * regionWidth;

        for (int i = 0; i < BoidsPerSwarm; i++)
        {
            float angle = (float)((double)rand() / RAND_MAX * M_PI * 2);
            float speed = MaxSpeed * (0.5f + (float)((double)rand() / RAND_MAX) * 0.5f);

            boids[index].Position = Vector2{
                regionStartX + (float)((double)rand() / RAND_MAX) * regionWidth,
                margin + (float)((double)rand() / RAND_MAX) * (WORLD_HEIGHT - 2 * margin)
            };
            boids[index].Velocity = Vector2{ cosf(angle) * speed, sinf(angle) * speed };
            boids[index].Acceleration = Vector2{ 0, 0 };
            boids[index].SwarmIndex = s;
            boids[index].Color = swarmColors[s];
            index++;
        }
    }
}

// ── Spatial Grid ─────────────────────────────────────────────────────────────
static void BuildSpatialGrid()
{
    memset(gridCellCounts, 0, sizeof(gridCellCounts));

    for (int i = 0; i < BoidCount; i++)
    {
        int cellIdx = GetCellIndex(boids[i].Position.x, boids[i].Position.y);
        int count = gridCellCounts[cellIdx];
        if (count < MaxBoidsPerCell)
        {
            gridCells[cellIdx * MaxBoidsPerCell + count] = i;
            gridCellCounts[cellIdx]++;
        }
    }
}

// ── Update ───────────────────────────────────────────────────────────────────
static void UpdateBoids(float dt)
{
    Vector2 mousePos = { 0, 0 };
    bool mouseActive = false;
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
    {
        mousePos = GetMousePosition();
        mousePos = GetScreenToWorld2D(mousePos, camera);
        mouseActive = true;
    }

    Vector2 rightClickPos = { 0, 0 };
    bool repelActive = false;
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
    {
        rightClickPos = GetMousePosition();
        rightClickPos = GetScreenToWorld2D(rightClickPos, camera);
        repelActive = true;
    }

    // First pass: compute accelerations using spatial grid
    for (int i = 0; i < BoidCount; i++)
    {
        Boid& boid = boids[i];
        float posX = boid.Position.x;
        float posY = boid.Position.y;
        int swarmIdx = boid.SwarmIndex;

        float sepX = 0, sepY = 0;
        float aliX = 0, aliY = 0;
        float cohX = 0, cohY = 0;
        int sepCount = 0;
        int neighborCount = 0;

        // Check neighboring cells (3x3 grid around the boid's cell)
        int cellIdx = GetCellIndex(posX, posY);
        int cellCol = cellIdx % GridCols;
        int cellRow = cellIdx / GridCols;

        int minCol = cellCol - 1; if (minCol < 0) minCol = 0;
        int maxCol = cellCol + 1; if (maxCol >= GridCols) maxCol = GridCols - 1;
        int minRow = cellRow - 1; if (minRow < 0) minRow = 0;
        int maxRow = cellRow + 1; if (maxRow >= GridRows) maxRow = GridRows - 1;

        for (int r = minRow; r <= maxRow; r++)
        {
            for (int c = minCol; c <= maxCol; c++)
            {
                int cellBase = (r * GridCols + c) * MaxBoidsPerCell;
                int count = gridCellCounts[r * GridCols + c];

                for (int k = 0; k < count; k++)
                {
                    int j = gridCells[cellBase + k];
                    if (j == i) continue;

                    Boid& other = boids[j];
                    float dx = posX - other.Position.x;
                    float dy = posY - other.Position.y;
                    float distSq = dx * dx + dy * dy;

                    if (distSq < PerceptionRadiusSq && distSq > 0.001f)
                    {
                        // Separation from all boids
                        if (distSq < SeparationRadiusSq)
                        {
                            float invDist = InvSqrt(distSq);
                            sepX += dx * invDist * invDist; // dx / dist^2
                            sepY += dy * invDist * invDist;
                            sepCount++;
                        }

                        // Alignment & cohesion only from same swarm
                        if (other.SwarmIndex == swarmIdx)
                        {
                            aliX += other.Velocity.x;
                            aliY += other.Velocity.y;
                            cohX += other.Position.x;
                            cohY += other.Position.y;
                            neighborCount++;
                        }
                    }
                }
            }
        }

        float accX = 0, accY = 0;

        // Separation force
        if (sepCount > 0)
        {
            float invSepCount = 1.0f / sepCount;
            float sx = sepX * invSepCount;
            float sy = sepY * invSepCount;
            float len = sqrtf(sx * sx + sy * sy);
            if (len > 0.001f)
            {
                float desiredX = sx / len * MaxSpeed - boid.Velocity.x;
                float desiredY = sy / len * MaxSpeed - boid.Velocity.y;
                float dLen = sqrtf(desiredX * desiredX + desiredY * desiredY);
                if (dLen > MaxForce)
                {
                    desiredX = desiredX / dLen * MaxForce;
                    desiredY = desiredY / dLen * MaxForce;
                }
                accX += desiredX * SeparationWeight;
                accY += desiredY * SeparationWeight;
            }
        }

        // Alignment force
        if (neighborCount > 0)
        {
            float invN = 1.0f / neighborCount;
            float ax = aliX * invN;
            float ay = aliY * invN;
            float len = sqrtf(ax * ax + ay * ay);
            if (len > 0.001f)
            {
                float desiredX = ax / len * MaxSpeed - boid.Velocity.x;
                float desiredY = ay / len * MaxSpeed - boid.Velocity.y;
                float dLen = sqrtf(desiredX * desiredX + desiredY * desiredY);
                if (dLen > MaxForce)
                {
                    desiredX = desiredX / dLen * MaxForce;
                    desiredY = desiredY / dLen * MaxForce;
                }
                accX += desiredX * AlignmentWeight;
                accY += desiredY * AlignmentWeight;
            }

            // Cohesion force
            float cx = cohX * invN - posX;
            float cy = cohY * invN - posY;
            float cLen = sqrtf(cx * cx + cy * cy);
            if (cLen > 0.001f)
            {
                float desiredX = cx / cLen * MaxSpeed - boid.Velocity.x;
                float desiredY = cy / cLen * MaxSpeed - boid.Velocity.y;
                float dLen = sqrtf(desiredX * desiredX + desiredY * desiredY);
                if (dLen > MaxForce)
                {
                    desiredX = desiredX / dLen * MaxForce;
                    desiredY = desiredY / dLen * MaxForce;
                }
                accX += desiredX * CohesionWeight;
                accY += desiredY * CohesionWeight;
            }
        }

        // Boundary avoidance
        if (posX < BoundaryMargin)
            accX += TurnForce;
        else if (posX > WORLD_WIDTH - BoundaryMargin)
            accX -= TurnForce;

        if (posY < BoundaryMargin)
            accY += TurnForce;
        else if (posY > WORLD_HEIGHT - BoundaryMargin)
            accY -= TurnForce;

        // Mouse interaction
        if (mouseActive)
        {
            float dx = mousePos.x - posX;
            float dy = mousePos.y - posY;
            float distSq = dx * dx + dy * dy;
            if (distSq < 22500.0f && distSq > 0.001f)
            {
                float invDist = InvSqrt(distSq);
                accX += dx * invDist * 200.0f;
                accY += dy * invDist * 200.0f;
            }
        }

        if (repelActive)
        {
            float dx = posX - rightClickPos.x;
            float dy = posY - rightClickPos.y;
            float distSq = dx * dx + dy * dy;
            if (distSq < 22500.0f && distSq > 0.001f)
            {
                float invDist = InvSqrt(distSq);
                accX += dx * invDist * 300.0f;
                accY += dy * invDist * 300.0f;
            }
        }

        boid.Acceleration = Vector2{ accX, accY };
    }

    // Second pass: integrate
    for (int i = 0; i < BoidCount; i++)
    {
        Boid& boid = boids[i];

        float vx = boid.Velocity.x + boid.Acceleration.x * dt;
        float vy = boid.Velocity.y + boid.Acceleration.y * dt;

        // Limit speed
        float speedSq = vx * vx + vy * vy;
        if (speedSq > MaxSpeed * MaxSpeed)
        {
            float invSpeed = InvSqrt(speedSq) * MaxSpeed;
            vx *= invSpeed;
            vy *= invSpeed;
        }

        float px = boid.Position.x + vx * dt;
        float py = boid.Position.y + vy * dt;

        // Wrap around (world bounds)
        if (px < -BoidSize) px = WORLD_WIDTH + BoidSize;
        else if (px > WORLD_WIDTH + BoidSize) px = -BoidSize;
        if (py < -BoidSize) py = WORLD_HEIGHT + BoidSize;
        else if (py > WORLD_HEIGHT + BoidSize) py = -BoidSize;

        boid.Velocity = Vector2{ vx, vy };
        boid.Position = Vector2{ px, py };
    }
}

// ── Drawing ──────────────────────────────────────────────────────────────────
static void DrawBorders()
{
    DrawRectangleLinesEx(
        Rectangle{ BoundaryMargin, BoundaryMargin, WORLD_WIDTH - 2 * BoundaryMargin, WORLD_HEIGHT - 2 * BoundaryMargin },
        2,
        Color{ 60, 80, 120, 255 });
}

static void DrawBoids()
{
    for (int i = 0; i < BoidCount; i++)
    {
        Boid& boid = boids[i];
        Vector2 pos = boid.Position;
        Vector2 vel = boid.Velocity;

        // Direction angle
        float angle = atan2f(vel.y, vel.x);
        float cosA = cosf(angle);
        float sinA = sinf(angle);

        // Tip
        float tipX = pos.x + cosA * TipOffset;
        float tipY = pos.y + sinA * TipOffset;

        // Left wing (angle + 2.5 rad)
        float cosLeftA = cosA * CosLeft - sinA * SinLeft;
        float sinLeftA = sinA * CosLeft + cosA * SinLeft;
        float leftX = pos.x + cosLeftA * BackOffset;
        float leftY = pos.y + sinLeftA * BackOffset;

        // Right wing (angle - 2.5 rad)
        float cosRightA = cosA * CosRight - sinA * SinRight;
        float sinRightA = sinA * CosRight + cosA * SinRight;
        float rightX = pos.x + cosRightA * BackOffset;
        float rightY = pos.y + sinRightA * BackOffset;

        DrawTriangle(
            Vector2{ tipX, tipY },
            Vector2{ rightX, rightY },
            Vector2{ leftX, leftY },
            boid.Color);
    }
}

// ── Main ─────────────────────────────────────────────────────────────────────
int main(void)
{
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(ScreenWidth, ScreenHeight, "Raylib C++ 2D Boids Flocking Simulation");
    SetTargetFPS(60);

    // Initialize camera centered on the world
    InitCamera();

    // Initialize boids
    srand(42);
    InitSwarmColors();
    InitBoids();

    // Mouse drag state
    bool isDragging = false;
    Vector2 dragStart = { 0, 0 };
    Vector2 camTargetAtDragStart = { 0, 0 };

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        if (dt > 0.033f) dt = 0.033f;

        // Handle window resize
        if (IsWindowResized())
        {
            ScreenWidth  = GetScreenWidth();
            ScreenHeight = GetScreenHeight();
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
            Vector2 mousePos = GetMousePosition();
            Vector2 worldPos = GetScreenToWorld2D(mousePos, camera);

            camera.zoom *= (1.0f + wheel * 0.1f);
            if (camera.zoom < 0.1f) camera.zoom = 0.1f;
            if (camera.zoom > 10.0f) camera.zoom = 10.0f;

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

        // Handle input (reset)
        if (IsKeyPressed(KEY_R))
        {
            InitBoids();
        }

        // Update simulation
        BuildSpatialGrid();
        UpdateBoids(dt);

        // Render
        BeginDrawing();
        ClearBackground(Color{ 15, 15, 25, 255 });

        // World-space rendering (affected by camera)
        BeginMode2D(camera);
        DrawBorders();
        DrawBoids();
        EndMode2D();

        // UI (screen-space, not affected by camera)
        int ls = 25;
        DrawFPS(10, 10);
        char buf[128];
        snprintf(buf, sizeof(buf), "Swarms: %d  |  Boids: %d", NumSwarms, BoidCount);
        DrawText(buf, 10, 10 + ls, 20, LIGHTGRAY);
        DrawText("R to reset  |  Click to attract  |  Right-click to repel", 10, 10 + 2 * ls, 20, LIGHTGRAY);
        DrawText("WASD/Arrows to pan  |  Mouse drag to pan  |  Scroll to zoom", 10, 10 + 3 * ls, 20, LIGHTGRAY);
        EndDrawing();
    }

    CloseWindow();

    return 0;
}