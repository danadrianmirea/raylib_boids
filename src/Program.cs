using Raylib_cs;
using System.Numerics;
using System.Runtime.CompilerServices;
using Color = Raylib_cs.Color;
using Rectangle = Raylib_cs.Rectangle;

namespace Raylib2DSwarm;

/// <summary>
/// Optimized 2D boids flocking simulation using triangular agents.
/// Uses spatial grid partitioning for O(n) neighbor lookups instead of O(n²).
/// </summary>
class Program
{
    // --- Window & Simulation Constants ---
    const int ScreenWidth = 960;
    const int ScreenHeight = 540;
    const int NumSwarms = 20;
    const int BoidsPerSwarm = 500;
    const int BoidCount = NumSwarms * BoidsPerSwarm;
    const float BoidSize = 12.0f;
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

    // Spatial grid constants
    const int GridCellSize = (int)PerceptionRadius;
    const int GridCols = ScreenWidth / GridCellSize + 2;
    const int GridRows = ScreenHeight / GridCellSize + 2;
    const int MaxBoidsPerCell = 64;

    struct Boid
    {
        public Vector2 Position;
        public Vector2 Velocity;
        public Vector2 Acceleration;
        public int SwarmIndex;
        public Color Color;
    }

    static Boid[] boids = new Boid[BoidCount];
    static Color[] swarmColors = new Color[NumSwarms];
    static Random rng = new Random();

    // Spatial grid: for each cell, store indices of boids in that cell
    static int[] gridCellCounts = new int[GridCols * GridRows];
    static int[] gridCells = new int[GridCols * GridRows * MaxBoidsPerCell];

    // Pre-computed triangle vertex offsets (avoid sin/cos per boid)
    static readonly float TipOffset = BoidSize * 0.6f;
    static readonly float BackOffset = BoidSize * 0.3f;
    static readonly float LeftAngle = 2.5f;
    static readonly float RightAngle = -2.5f;
    static readonly float CosLeft = MathF.Cos(LeftAngle);
    static readonly float SinLeft = MathF.Sin(LeftAngle);
    static readonly float CosRight = MathF.Cos(RightAngle);
    static readonly float SinRight = MathF.Sin(RightAngle);

    static void Main(string[] args)
    {
        Raylib.InitWindow(ScreenWidth, ScreenHeight, "Raylib C# 2D Boids Flocking Simulation");
        Raylib.SetTargetFPS(60);

        InitSwarmColors();
        InitBoids();

        while (!Raylib.WindowShouldClose())
        {
            float dt = Raylib.GetFrameTime();
            if (dt > 0.033f) dt = 0.033f;

            HandleInput();

            // Update
            BuildSpatialGrid();
            UpdateBoids(dt);

            // Render
            Raylib.BeginDrawing();
            Raylib.ClearBackground(new Color(10, 10, 20, 255));

            DrawBorders();
            DrawBoids();

            // UI
            int ls = 25;
            Raylib.DrawFPS(10, 10);
            Raylib.DrawText($"Swarms: {NumSwarms}  |  Boids: {BoidCount}", 10, 10 + ls, 20, Color.LightGray);
            Raylib.DrawText("R to reset  |  Click to attract  |  Right-click to repel", 10, 10 + 2 * ls, 20, Color.LightGray);

            Raylib.EndDrawing();
        }

        Raylib.CloseWindow();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

    static void BuildSpatialGrid()
    {
        Array.Clear(gridCellCounts, 0, gridCellCounts.Length);

        for (int i = 0; i < BoidCount; i++)
        {
            int cellIdx = GetCellIndex(boids[i].Position.X, boids[i].Position.Y);
            int count = gridCellCounts[cellIdx];
            if (count < MaxBoidsPerCell)
            {
                gridCells[cellIdx * MaxBoidsPerCell + count] = i;
                gridCellCounts[cellIdx]++;
            }
        }
    }

    static void InitSwarmColors()
    {
        for (int s = 0; s < NumSwarms; s++)
        {
            float hue = (float)s / NumSwarms * 360.0f;
            swarmColors[s] = ColorFromHSV(hue, 0.85f, 0.95f);
        }
    }

    static void HandleInput()
    {
        if (Raylib.IsKeyPressed(KeyboardKey.R))
        {
            InitBoids();
        }
    }

    static void InitBoids()
    {
        float margin = 80.0f;
        int index = 0;

        for (int s = 0; s < NumSwarms; s++)
        {
            float regionWidth = (ScreenWidth - 2 * margin) / NumSwarms;
            float regionStartX = margin + s * regionWidth;

            for (int i = 0; i < BoidsPerSwarm; i++)
            {
                float angle = (float)(rng.NextDouble() * Math.PI * 2);
                float speed = MaxSpeed * (0.5f + (float)rng.NextDouble() * 0.5f);

                boids[index] = new Boid
                {
                    Position = new Vector2(
                        regionStartX + (float)rng.NextDouble() * regionWidth,
                        margin + (float)rng.NextDouble() * (ScreenHeight - 2 * margin)
                    ),
                    Velocity = new Vector2(MathF.Cos(angle) * speed, MathF.Sin(angle) * speed),
                    Acceleration = Vector2.Zero,
                    SwarmIndex = s,
                    Color = swarmColors[s]
                };
                index++;
            }
        }
    }

    static Color ColorFromHSV(float hue, float saturation, float value)
    {
        float c = value * saturation;
        float x = c * (1.0f - MathF.Abs((hue / 60.0f) % 2.0f - 1.0f));
        float m = value - c;

        float r, g, b;
        if (hue < 60) { r = c; g = x; b = 0; }
        else if (hue < 120) { r = x; g = c; b = 0; }
        else if (hue < 180) { r = 0; g = c; b = x; }
        else if (hue < 240) { r = 0; g = x; b = c; }
        else if (hue < 300) { r = x; g = 0; b = c; }
        else { r = c; g = 0; b = x; }

        return new Color(
            (int)((r + m) * 255),
            (int)((g + m) * 255),
            (int)((b + m) * 255),
            255
        );
    }

    static void DrawBorders()
    {
        Raylib.DrawRectangleLinesEx(
            new Rectangle(BoundaryMargin, BoundaryMargin, ScreenWidth - 2 * BoundaryMargin, ScreenHeight - 2 * BoundaryMargin),
            2,
            new Color(60, 80, 120, 255));
    }

    static void DrawBoids()
    {
        for (int i = 0; i < BoidCount; i++)
        {
            ref Boid boid = ref boids[i];
            Vector2 pos = boid.Position;
            Vector2 vel = boid.Velocity;

            // Direction angle
            float angle = MathF.Atan2(vel.Y, vel.X);
            float cosA = MathF.Cos(angle);
            float sinA = MathF.Sin(angle);

            // Tip
            float tipX = pos.X + cosA * TipOffset;
            float tipY = pos.Y + sinA * TipOffset;

            // Left wing (angle + 2.5 rad)
            float cosLeftA = cosA * CosLeft - sinA * SinLeft;
            float sinLeftA = sinA * CosLeft + cosA * SinLeft;
            float leftX = pos.X + cosLeftA * BackOffset;
            float leftY = pos.Y + sinLeftA * BackOffset;

            // Right wing (angle - 2.5 rad)
            float cosRightA = cosA * CosRight - sinA * SinRight;
            float sinRightA = sinA * CosRight + cosA * SinRight;
            float rightX = pos.X + cosRightA * BackOffset;
            float rightY = pos.Y + sinRightA * BackOffset;

            Raylib.DrawTriangle(
                new Vector2(tipX, tipY),
                new Vector2(rightX, rightY),
                new Vector2(leftX, leftY),
                boid.Color);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    static float InvSqrt(float x)
    {
        return 1.0f / MathF.Sqrt(x);
    }

    static void UpdateBoids(float dt)
    {
        Vector2? mousePos = null;
        if (Raylib.IsMouseButtonDown(MouseButton.Left))
            mousePos = Raylib.GetMousePosition();

        Vector2? rightClickPos = null;
        if (Raylib.IsMouseButtonDown(MouseButton.Right))
            rightClickPos = Raylib.GetMousePosition();

        float mouseAttractX = 0, mouseAttractY = 0;
        bool mouseActive = false;
        if (mousePos.HasValue) { mouseAttractX = mousePos.Value.X; mouseAttractY = mousePos.Value.Y; mouseActive = true; }

        float mouseRepelX = 0, mouseRepelY = 0;
        bool repelActive = false;
        if (rightClickPos.HasValue) { mouseRepelX = rightClickPos.Value.X; mouseRepelY = rightClickPos.Value.Y; repelActive = true; }

        // First pass: compute accelerations using spatial grid
        for (int i = 0; i < BoidCount; i++)
        {
            ref Boid boid = ref boids[i];
            float posX = boid.Position.X;
            float posY = boid.Position.Y;
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

                        ref Boid other = ref boids[j];
                        float dx = posX - other.Position.X;
                        float dy = posY - other.Position.Y;
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
                                aliX += other.Velocity.X;
                                aliY += other.Velocity.Y;
                                cohX += other.Position.X;
                                cohY += other.Position.Y;
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
                float len = MathF.Sqrt(sx * sx + sy * sy);
                if (len > 0.001f)
                {
                    float desiredX = sx / len * MaxSpeed - boid.Velocity.X;
                    float desiredY = sy / len * MaxSpeed - boid.Velocity.Y;
                    float dLen = MathF.Sqrt(desiredX * desiredX + desiredY * desiredY);
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
                float len = MathF.Sqrt(ax * ax + ay * ay);
                if (len > 0.001f)
                {
                    float desiredX = ax / len * MaxSpeed - boid.Velocity.X;
                    float desiredY = ay / len * MaxSpeed - boid.Velocity.Y;
                    float dLen = MathF.Sqrt(desiredX * desiredX + desiredY * desiredY);
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
                float cLen = MathF.Sqrt(cx * cx + cy * cy);
                if (cLen > 0.001f)
                {
                    float desiredX = cx / cLen * MaxSpeed - boid.Velocity.X;
                    float desiredY = cy / cLen * MaxSpeed - boid.Velocity.Y;
                    float dLen = MathF.Sqrt(desiredX * desiredX + desiredY * desiredY);
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
            else if (posX > ScreenWidth - BoundaryMargin)
                accX -= TurnForce;

            if (posY < BoundaryMargin)
                accY += TurnForce;
            else if (posY > ScreenHeight - BoundaryMargin)
                accY -= TurnForce;

            // Mouse interaction
            if (mouseActive)
            {
                float dx = mouseAttractX - posX;
                float dy = mouseAttractY - posY;
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
                float dx = posX - mouseRepelX;
                float dy = posY - mouseRepelY;
                float distSq = dx * dx + dy * dy;
                if (distSq < 22500.0f && distSq > 0.001f)
                {
                    float invDist = InvSqrt(distSq);
                    accX += dx * invDist * 300.0f;
                    accY += dy * invDist * 300.0f;
                }
            }

            boid.Acceleration = new Vector2(accX, accY);
        }

        // Second pass: integrate
        for (int i = 0; i < BoidCount; i++)
        {
            ref Boid boid = ref boids[i];

            float vx = boid.Velocity.X + boid.Acceleration.X * dt;
            float vy = boid.Velocity.Y + boid.Acceleration.Y * dt;

            // Limit speed
            float speedSq = vx * vx + vy * vy;
            if (speedSq > MaxSpeed * MaxSpeed)
            {
                float invSpeed = InvSqrt(speedSq) * MaxSpeed;
                vx *= invSpeed;
                vy *= invSpeed;
            }

            float px = boid.Position.X + vx * dt;
            float py = boid.Position.Y + vy * dt;

            // Wrap around
            if (px < -BoidSize) px = ScreenWidth + BoidSize;
            else if (px > ScreenWidth + BoidSize) px = -BoidSize;
            if (py < -BoidSize) py = ScreenHeight + BoidSize;
            else if (py > ScreenHeight + BoidSize) py = -BoidSize;

            boid.Velocity = new Vector2(vx, vy);
            boid.Position = new Vector2(px, py);
        }
    }
}
