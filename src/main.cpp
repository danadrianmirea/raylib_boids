#include <raylib.h>
#include <rlgl.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <ctime>

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
const int BoidCount = 4500;
const float BoidSize = 16.0f;

// Runtime parameters (randomized on reset)
static int NumSwarms = 3;
static int BoidsPerSwarm = BoidCount / NumSwarms;
static float MaxSpeed = 200.0f;
static float MaxSpeedSq = MaxSpeed * MaxSpeed;
static float MaxForce = 100.0f;
static float MaxForceSq = MaxForce * MaxForce;
static float PerceptionRadius = 80.0f;
static float PerceptionRadiusSq = PerceptionRadius * PerceptionRadius;
static float SeparationRadius = 30.0f;
static float SeparationRadiusSq = SeparationRadius * SeparationRadius;

// Boid rule weights
const float SeparationWeight = 1.5f;
const float AlignmentWeight = 1.0f;
const float CohesionWeight = 1.0f;

// Boundary behavior
const float BoundaryMargin = 40.0f;
const float TurnForce = 300.0f;

// Spatial grid constants (based on max possible perception radius)
const int GridCellSize = 120; // max perception radius
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
static Color swarmColors[6]; // max possible swarms

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
static void ResetSimulation();
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

// ── Reset / Randomize ────────────────────────────────────────────────────────
static void ResetSimulation()
{
    // Randomize number of swarms (2 to 6)
    NumSwarms = 3 + rand() % 4; // 2, 3, 4, 5, or 6
    BoidsPerSwarm = BoidCount / NumSwarms;

    // Randomize movement parameters within sensible ranges
    MaxSpeed = 200.0f + (float)(rand() % 201);
    MaxSpeedSq = MaxSpeed * MaxSpeed;
    MaxForce = 50.0f + (float)(rand() % 151);
    MaxForceSq = MaxForce * MaxForce;
    PerceptionRadius = 50.0f + (float)(rand() % 71);
    PerceptionRadiusSq = PerceptionRadius * PerceptionRadius;
    SeparationRadius = 15.0f + (float)(rand() % 36);
    SeparationRadiusSq = SeparationRadius * SeparationRadius;

    InitSwarmColors();
    InitBoids();
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
    float mouseX = 0, mouseY = 0;
    bool mouseActive = false;
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT))
    {
        Vector2 mp = GetMousePosition();
        mp = GetScreenToWorld2D(mp, camera);
        mouseX = mp.x;
        mouseY = mp.y;
        mouseActive = true;
    }

    float repelX = 0, repelY = 0;
    bool repelActive = false;
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT))
    {
        Vector2 rp = GetMousePosition();
        rp = GetScreenToWorld2D(rp, camera);
        repelX = rp.x;
        repelY = rp.y;
        repelActive = true;
    }

    // Cache constants locally for better codegen
    const float maxSpeed = MaxSpeed;
    const float maxSpeedSq = MaxSpeedSq;
    const float maxForce = MaxForce;
    const float maxForceSq = MaxForceSq;
    const float perceptionRadiusSq = PerceptionRadiusSq;
    const float separationRadiusSq = SeparationRadiusSq;
    const float sepWeight = SeparationWeight;
    const float aliWeight = AlignmentWeight;
    const float cohWeight = CohesionWeight;
    const float turnForce = TurnForce;
    const float boundaryMargin = BoundaryMargin;
    const float worldW = WORLD_WIDTH;
    const float worldH = WORLD_HEIGHT;
    const float boidSize = BoidSize;

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
            int rowOffset = r * GridCols;
            for (int c = minCol; c <= maxCol; c++)
            {
                int cellBase = (rowOffset + c) * MaxBoidsPerCell;
                int count = gridCellCounts[rowOffset + c];

                for (int k = 0; k < count; k++)
                {
                    int j = gridCells[cellBase + k];
                    if (j == i) continue;

                    const Boid& other = boids[j];
                    float dx = posX - other.Position.x;
                    float dy = posY - other.Position.y;
                    float distSq = dx * dx + dy * dy;

                    if (distSq < perceptionRadiusSq && distSq > 0.001f)
                    {
                        // Separation from all boids
                        if (distSq < separationRadiusSq)
                        {
                            float invDistSq = 1.0f / distSq; // 1/dist^2
                            sepX += dx * invDistSq;
                            sepY += dy * invDistSq;
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
            float lenSq = sx * sx + sy * sy;
            if (lenSq > 0.0001f)
            {
                // Normalize and scale to maxSpeed, then subtract velocity
                float invLen = InvSqrt(lenSq);
                float desiredX = sx * invLen * maxSpeed - boid.Velocity.x;
                float desiredY = sy * invLen * maxSpeed - boid.Velocity.y;
                // Limit force using squared comparison
                float dLenSq = desiredX * desiredX + desiredY * desiredY;
                if (dLenSq > maxForceSq)
                {
                    float invDLen = InvSqrt(dLenSq) * maxForce;
                    desiredX *= invDLen;
                    desiredY *= invDLen;
                }
                accX += desiredX * sepWeight;
                accY += desiredY * sepWeight;
            }
        }

        // Alignment & Cohesion
        if (neighborCount > 0)
        {
            float invN = 1.0f / neighborCount;

            // Alignment
            float ax = aliX * invN;
            float ay = aliY * invN;
            float aLenSq = ax * ax + ay * ay;
            if (aLenSq > 0.0001f)
            {
                float invALen = InvSqrt(aLenSq);
                float desiredX = ax * invALen * maxSpeed - boid.Velocity.x;
                float desiredY = ay * invALen * maxSpeed - boid.Velocity.y;
                float dLenSq = desiredX * desiredX + desiredY * desiredY;
                if (dLenSq > maxForceSq)
                {
                    float invDLen = InvSqrt(dLenSq) * maxForce;
                    desiredX *= invDLen;
                    desiredY *= invDLen;
                }
                accX += desiredX * aliWeight;
                accY += desiredY * aliWeight;
            }

            // Cohesion
            float cx = cohX * invN - posX;
            float cy = cohY * invN - posY;
            float cLenSq = cx * cx + cy * cy;
            if (cLenSq > 0.0001f)
            {
                float invCLen = InvSqrt(cLenSq);
                float desiredX = cx * invCLen * maxSpeed - boid.Velocity.x;
                float desiredY = cy * invCLen * maxSpeed - boid.Velocity.y;
                float dLenSq = desiredX * desiredX + desiredY * desiredY;
                if (dLenSq > maxForceSq)
                {
                    float invDLen = InvSqrt(dLenSq) * maxForce;
                    desiredX *= invDLen;
                    desiredY *= invDLen;
                }
                accX += desiredX * cohWeight;
                accY += desiredY * cohWeight;
            }
        }

        // Boundary avoidance
        if (posX < boundaryMargin)
            accX += turnForce;
        else if (posX > worldW - boundaryMargin)
            accX -= turnForce;

        if (posY < boundaryMargin)
            accY += turnForce;
        else if (posY > worldH - boundaryMargin)
            accY -= turnForce;

        // Mouse interaction
        if (mouseActive)
        {
            float dx = mouseX - posX;
            float dy = mouseY - posY;
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
            float dx = posX - repelX;
            float dy = posY - repelY;
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

        // Limit speed using pre-computed MaxSpeedSq
        float speedSq = vx * vx + vy * vy;
        if (speedSq > maxSpeedSq)
        {
            float invSpeed = InvSqrt(speedSq) * maxSpeed;
            vx *= invSpeed;
            vy *= invSpeed;
        }

        float px = boid.Position.x + vx * dt;
        float py = boid.Position.y + vy * dt;

        // Wrap around (world bounds)
        if (px < -boidSize) px = worldW + boidSize;
        else if (px > worldW + boidSize) px = -boidSize;
        if (py < -boidSize) py = worldH + boidSize;
        else if (py > worldH + boidSize) py = -boidSize;

        boid.Velocity = Vector2{ vx, vy };
        boid.Position = Vector2{ px, py };
    }
}

// ── Timing Statistics ────────────────────────────────────────────────────────
static double updateTimeMs = 0.0;
static double drawTimeMs = 0.0;

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
    // Batch all boid triangles into a single draw call using rlgl
    rlBegin(RL_TRIANGLES);

    for (int i = 0; i < BoidCount; i++)
    {
        const Boid& boid = boids[i];
        float posX = boid.Position.x;
        float posY = boid.Position.y;
        float vx = boid.Velocity.x;
        float vy = boid.Velocity.y;

        // Direction angle
        float cosA, sinA;
        float lenSq = vx * vx + vy * vy;
        if (lenSq > 0.0001f)
        {
            float invLen = InvSqrt(lenSq);
            cosA = vx * invLen;
            sinA = vy * invLen;
        }
        else
        {
            cosA = 1.0f;
            sinA = 0.0f;
        }

        // Tip
        float tipX = posX + cosA * TipOffset;
        float tipY = posY + sinA * TipOffset;

        // Left wing (angle + 2.5 rad)
        float cosLeftA = cosA * CosLeft - sinA * SinLeft;
        float sinLeftA = sinA * CosLeft + cosA * SinLeft;
        float leftX = posX + cosLeftA * BackOffset;
        float leftY = posY + sinLeftA * BackOffset;

        // Right wing (angle - 2.5 rad)
        float cosRightA = cosA * CosRight - sinA * SinRight;
        float sinRightA = sinA * CosRight + cosA * SinRight;
        float rightX = posX + cosRightA * BackOffset;
        float rightY = posY + sinRightA * BackOffset;

        // Set color once per triangle
        rlColor4ub(boid.Color.r, boid.Color.g, boid.Color.b, boid.Color.a);

        rlVertex2f(tipX, tipY);
        rlVertex2f(rightX, rightY);
        rlVertex2f(leftX, leftY);
    }

    rlEnd();
}


// ── Main ─────────────────────────────────────────────────────────────────────
int main(void)
{
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(ScreenWidth, ScreenHeight, "Raylib C++ 2D Boids Flocking Simulation");
    SetTargetFPS(60);

    // Initialize camera centered on the world
    InitCamera();

    // Initialize boids with randomized parameters
    srand(time(0));
    ResetSimulation();

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
            ResetSimulation();
        }

        // Update simulation with timing
        double t0 = GetTime();
        BuildSpatialGrid();
        UpdateBoids(dt);
        double t1 = GetTime();
        updateTimeMs = (t1 - t0) * 1000.0;

        // Render with timing
        BeginDrawing();
        ClearBackground(Color{ 15, 15, 25, 255 });

        // World-space rendering (affected by camera)
        BeginMode2D(camera);
        DrawBorders();
        double t2 = GetTime();
        DrawBoids();
        double t3 = GetTime();
        drawTimeMs = (t3 - t2) * 1000.0;
        EndMode2D();

        // UI (screen-space, not affected by camera)
        int ls = 20;
        DrawFPS(10, 10);
        char buf[256];
        snprintf(buf, sizeof(buf), "Swarms: %d  |  Boids: %d  |  Speed: %.0f  |  Force: %.0f",
            NumSwarms, BoidCount, MaxSpeed, MaxForce);
        DrawText(buf, 10, 10 + ls, 18, LIGHTGRAY);
        snprintf(buf, sizeof(buf), "Perception: %.0f  |  Separation: %.0f",
            PerceptionRadius, SeparationRadius);
        DrawText(buf, 10, 10 + 2 * ls, 18, LIGHTGRAY);
        snprintf(buf, sizeof(buf), "Update: %.2f ms  |  Draw: %.2f ms",
            updateTimeMs, drawTimeMs);
        DrawText(buf, 10, 10 + 3 * ls, 18, LIGHTGRAY);
        DrawText("R to randomize  |  Click to attract  |  Right-click to repel", 10, 10 + 4 * ls, 18, LIGHTGRAY);
        DrawText("WASD/Arrows to pan  |  Mouse drag to pan  |  Scroll to zoom", 10, 10 + 5 * ls, 18, LIGHTGRAY);
        EndDrawing();
    }

    CloseWindow();

    return 0;
}