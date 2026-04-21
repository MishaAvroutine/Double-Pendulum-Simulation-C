#define RAYGUI_IMPLEMENTATION
#include"../include/raygui.h"
#include<stdio.h>
#include"../include/raylib.h"
#include"../include/raymath.h"
#include"../include/style_jungle.h"
#include<math.h>


#define HEIGHT 800
#define WIDTH 1280


#define THICKNESS_OF_LINE 10
#define MASS_RADIUS 20


#define L1 250.0
#define L2 200.0
#define DT 0.01
#define DAMPEN 0.999


#define MIN_L 1.0
#define MAX_L 400.0


#define MIN_G 1.0
#define MAX_G 1000.0

#define MIN_MASS 1.0
#define MAX_MASS 100.0

#define SLIDER_BAR_X_MARGIN 50
#define SLIDER_BAR_WIDTH 200
#define SLIDER_BAR_HEIGHT 25


#define FONT_SIZE 20

#define MARGIN 25

static float G = 100.0;
static bool dampen = false;
static bool isPaused = false;
static float speedMultiplier = 1.0f;
static float initialAlpha1 = 0.0f;
static float initialAlpha2 = 0.0f;

typedef struct Statistics
{
    float minAlpha1;
    float maxAlpha1;
    float minAlpha2;
    float maxAlpha2;
    float minVel1;
    float maxVel1;
    float minVel2;
    float maxVel2;
    float totalEnergy;
} Statistics;

typedef struct PendulumComponents
{
    float alpha1;
    float alpha2;

    float m1;
    float m2;

    float l1;
    float l2;

    float alpha1_d;
    float alpha1_dd;

    float alpha2_d;
    float alpha2_dd;
} PendulumComponents;

typedef struct MoveableBox
{
    Vector2 position;
    float width;
    float height;
    bool isDragging;
} MoveableBox;




Vector2 get_end(Vector2* start,float alpha,float l)
{
    Vector2 vec = {start->x + l * sinf(alpha),
    start->y + l * cosf(alpha)};
    return vec;
}


void drawPendulum(Vector2* startPos,float alpha,float l)
{
    Vector2 endPos = {
    startPos->x + l * sinf(alpha), // V.x
    startPos->y + l * cosf(alpha) // V.y
    };
    DrawLineEx(*startPos,endPos,THICKNESS_OF_LINE,WHITE);
    DrawCircleV(endPos,MASS_RADIUS,RED);
}


void drawDoublePendulum(Vector2* startPos,PendulumComponents* components,MoveableBox* box)
{
    Vector2 updatedStartPos = {startPos->x + box->width / 2, startPos->y + box->height / 2};
    Vector2 end_first = get_end(&updatedStartPos,components->alpha1,components->l1);
    drawPendulum(&end_first,components->alpha2,components->l2);
    drawPendulum(&updatedStartPos,components->alpha1,components->l1);
}

void drawMoveableBox(MoveableBox* box)
{
    DrawRectangleV(box->position, (Vector2){box->width, box->height}, BLUE);
    DrawRectangleLinesEx((Rectangle){box->position.x, box->position.y, box->width, box->height}, 2, WHITE);
}

void updateMoveableBox(MoveableBox* box)
{
    Rectangle boxRect = {box->position.x, box->position.y, box->width, box->height};
    Vector2 mousePos = GetMousePosition();
    
    // Check if mouse is over the box
    if (CheckCollisionPointRec(mousePos, boxRect) && IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        box->isDragging = true;
    }
    
    // If dragging, update position
    if (box->isDragging && IsMouseButtonDown(MOUSE_BUTTON_LEFT))
    {
        box->position = mousePos;
    }
    
    // Stop dragging when mouse is released
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
    {
        box->isDragging = false;
    }
}

void InitState(PendulumComponents* components)
{
    components->m1 = 20.0;
    components->alpha1 = initialAlpha1;
    components->alpha1_d = 0;
    components->alpha1_dd = 0;
    components->l1 = L1;

    components->m2 = 20.0;
    components->alpha2 = initialAlpha2;
    components->alpha2_d = 0;
    components->alpha2_dd = 0;
    components->l2 = L2;
}



float calculateTotalEnergy(Vector2* startPos, PendulumComponents* components, MoveableBox* box)
{
    Vector2 updatedStartPos = {startPos->x + box->width / 2, startPos->y + box->height / 2};
    Vector2 end1 = get_end(&updatedStartPos, components->alpha1, components->l1);
    Vector2 end2 = get_end(&end1, components->alpha2, components->l2);
    
    float v1 = components->l1 * components->alpha1_d;
    float v2_tangent = components->l2 * components->alpha2_d;
    float ke1 = 0.5f * components->m1 * v1 * v1;
    float ke2 = 0.5f * components->m2 * (v2_tangent * v2_tangent + v1 * v1);
    float ke = ke1 + ke2;
    
    float pe1 = components->m1 * G * (updatedStartPos.y - end1.y);
    float pe2 = components->m2 * G * (updatedStartPos.y - end2.y);
    float pe = pe1 + pe2;
    
    return ke + pe;
}

void updateStatistics(Statistics* stats, PendulumComponents* components, Vector2* startPos, MoveableBox* box)
{
    if (components->alpha1 < stats->minAlpha1) stats->minAlpha1 = components->alpha1;
    if (components->alpha1 > stats->maxAlpha1) stats->maxAlpha1 = components->alpha1;
    if (components->alpha2 < stats->minAlpha2) stats->minAlpha2 = components->alpha2;
    if (components->alpha2 > stats->maxAlpha2) stats->maxAlpha2 = components->alpha2;
    if (components->alpha1_d < stats->minVel1) stats->minVel1 = components->alpha1_d;
    if (components->alpha1_d > stats->maxVel1) stats->maxVel1 = components->alpha1_d;
    if (components->alpha2_d < stats->minVel2) stats->minVel2 = components->alpha2_d;
    if (components->alpha2_d > stats->maxVel2) stats->maxVel2 = components->alpha2_d;
    stats->totalEnergy = calculateTotalEnergy(startPos, components, box);
}


void solvePendulum(PendulumComponents* components, float dt)
{
    float sin_a1 = sinf(components->alpha1);
    float cos_a1 = cosf(components->alpha1);
    float sin_a2 = sinf(components->alpha2);
    float cos_a2 = cosf(components->alpha2);
    float diff = components->alpha1 - components->alpha2;
    float sin_diff = sinf(diff);
    float cos_diff = cosf(diff);
    float sin_2diff = sinf(2 * diff);
    float cos_2diff = cosf(2 * diff);
    
    float m1 = components->m1;
    float m2 = components->m2;
    float l1 = components->l1;
    float l2 = components->l2;
    float a1d_sq = components->alpha1_d * components->alpha1_d;
    float a2d_sq = components->alpha2_d * components->alpha2_d;
    
   
    float denom = 2 * m1 + m2 - m2 * cos_2diff;
    
    // θ1'' = −g (2 m1 + m2) sin θ1 − m2 g sin(θ1 − 2 θ2) − 2 sin(θ1 − θ2) m2 (θ2'2 L2 + θ1'2 L1 cos(θ1 − θ2))
    //        / L1 (2 m1 + m2 − m2 cos(2 θ1 − 2 θ2))
    float numerator1 = -G * (2 * m1 + m2) * sin_a1 
                       - m2 * G * sinf(components->alpha1 - 2 * components->alpha2)
                       - 2 * sin_diff * m2 * (a2d_sq * l2 + a1d_sq * l1 * cos_diff);
    components->alpha1_dd = numerator1 / (l1 * denom);
    
    // θ2'' = 2 sin(θ1 − θ2) (θ1'2 L1 (m1 + m2) + g(m1 + m2) cos θ1 + θ2'2 L2 m2 cos(θ1 − θ2))
    //        / L2 (2 m1 + m2 − m2 cos(2 θ1 − 2 θ2))
    float numerator2 = 2 * sin_diff * (a1d_sq * l1 * (m1 + m2) + G * (m1 + m2) * cos_a1 + a2d_sq * l2 * m2 * cos_diff);
    components->alpha2_dd = numerator2 / (l2 * denom);
    
    
    components->alpha1_d += components->alpha1_dd * dt;
    components->alpha2_d += components->alpha2_dd * dt;
    
    components->alpha1 += components->alpha1_d * dt;
    components->alpha2 += components->alpha2_d * dt;
    if(dampen)
    {
        components->alpha1_d *= DAMPEN;
        components->alpha2_d *= DAMPEN;
    }
}


void DrawUI(PendulumComponents* components, Statistics* stats)
{
    int col1_x = 10;
    int col2_x = 350;
    int col3_x = 690;
    int y_start = HEIGHT - MARGIN;
    int y_offset = MARGIN + 5;
    
    DrawText("=== PHYSICS ===", col1_x, y_start, FONT_SIZE, YELLOW);
    DrawText("G: ", col1_x, y_start - y_offset, FONT_SIZE, BLUE);
    GuiSliderBar((Rectangle){col1_x + 50, y_start - y_offset, SLIDER_BAR_WIDTH, SLIDER_BAR_HEIGHT}, "0.0", "1000.0", &G, MIN_G, MAX_G);
    
    DrawText("L1: ", col1_x, y_start - y_offset * 2, FONT_SIZE, BLUE);
    GuiSliderBar((Rectangle){col1_x + 50, y_start - y_offset * 2, SLIDER_BAR_WIDTH, SLIDER_BAR_HEIGHT}, "0.0", "400.0", &components->l1, MIN_L, MAX_L);
    
    DrawText("L2: ", col1_x, y_start - y_offset * 3, FONT_SIZE, BLUE);
    GuiSliderBar((Rectangle){col1_x + 50, y_start - y_offset * 3, SLIDER_BAR_WIDTH, SLIDER_BAR_HEIGHT}, "0.0", "400.0", &components->l2, MIN_L, MAX_L);
    
    DrawText("M1: ", col1_x, y_start - y_offset * 4, FONT_SIZE, BLUE);
    GuiSliderBar((Rectangle){col1_x + 50, y_start - y_offset * 4, SLIDER_BAR_WIDTH, SLIDER_BAR_HEIGHT}, "0.0", "100.0", &components->m1, MIN_MASS, MAX_MASS);
    
    DrawText("M2: ", col1_x, y_start - y_offset * 5, FONT_SIZE, BLUE);
    GuiSliderBar((Rectangle){col1_x + 50, y_start - y_offset * 5, SLIDER_BAR_WIDTH, SLIDER_BAR_HEIGHT}, "0.0", "100.0", &components->m2, MIN_MASS, MAX_MASS);
    
    DrawText("=== INIT ANGLES ===", col2_x, y_start, FONT_SIZE, YELLOW);
    DrawText("a1: ", col2_x, y_start - y_offset, FONT_SIZE, BLUE);
    GuiSliderBar((Rectangle){col2_x + 50, y_start - y_offset, SLIDER_BAR_WIDTH, SLIDER_BAR_HEIGHT}, "-180", "180", &initialAlpha1, -180, 180);
    
    DrawText("a2: ", col2_x, y_start - y_offset * 2, FONT_SIZE, BLUE);
    GuiSliderBar((Rectangle){col2_x + 50, y_start - y_offset * 2, SLIDER_BAR_WIDTH, SLIDER_BAR_HEIGHT}, "-180", "180", &initialAlpha2, -180, 180);
    
    DrawText("Speed: ", col2_x, y_start - y_offset * 3, FONT_SIZE, BLUE);
    GuiSliderBar((Rectangle){col2_x + 50, y_start - y_offset * 3, SLIDER_BAR_WIDTH, SLIDER_BAR_HEIGHT}, "0.1", "5.0", &speedMultiplier, 0.1f, 5.0f);
    
    bool dampen_copy = dampen;
    DrawText("Damp: ", col2_x, y_start - y_offset * 4, FONT_SIZE, BLUE);
    GuiCheckBox((Rectangle){col2_x + 50, y_start - y_offset * 4, 30, 25}, dampen_copy ? "ON" : "OFF", &dampen_copy);
    dampen = dampen_copy;
    
    DrawText("=== STATE ===", col3_x, y_start, FONT_SIZE, YELLOW);
    char buf[256];
    snprintf(buf, sizeof(buf), "a1: %.2f rad (%.1f°)", components->alpha1, components->alpha1 * RAD2DEG);
    DrawText(buf, col3_x, y_start - y_offset, FONT_SIZE, GREEN);
    
    snprintf(buf, sizeof(buf), "a2: %.2f rad (%.1f°)", components->alpha2, components->alpha2 * RAD2DEG);
    DrawText(buf, col3_x, y_start - y_offset * 2, FONT_SIZE, GREEN);
    
    snprintf(buf, sizeof(buf), "w1: %.3f rad/s", components->alpha1_d);
    DrawText(buf, col3_x, y_start - y_offset * 3, FONT_SIZE, GREEN);
    
    snprintf(buf, sizeof(buf), "w2: %.3f rad/s", components->alpha2_d);
    DrawText(buf, col3_x, y_start - y_offset * 4, FONT_SIZE, GREEN);
    
    snprintf(buf, sizeof(buf), "E: %.1f J", stats->totalEnergy);
    DrawText(buf, col3_x, y_start - y_offset * 5, FONT_SIZE, GREEN);
    
    const char* status = isPaused ? "[PAUSED] Press SPACE to resume" : "Press SPACE to pause";
    DrawText(status, WIDTH / 2 - 150, 10, FONT_SIZE, isPaused ? RED : WHITE);
    
    DrawText("R: Reset | Escape: Exit", 10, 35, 16, GRAY);
}



int main()
{
    InitWindow(WIDTH, HEIGHT, "Double Pendulum Simulation");
    GuiLoadStyleJungle();
    
    PendulumComponents components = {0};
    Statistics stats = {
        .minAlpha1 = 0,
        .maxAlpha1 = 0,
        .minAlpha2 = 0,
        .maxAlpha2 = 0,
        .minVel1 = 0,
        .maxVel1 = 0,
        .minVel2 = 0,
        .maxVel2 = 0,
        .totalEnergy = 0
    };
    
    initialAlpha1 = -45 * DEG2RAD;
    initialAlpha2 = -45 * DEG2RAD;
    InitState(&components);

    MoveableBox box = {0};
    box.position = (Vector2){WIDTH / 2, 50};
    box.width = 30;
    box.height = 30;
    box.isDragging = false;

    SetTargetFPS(120);
    
    while (!WindowShouldClose())
    {
        updateMoveableBox(&box);
        
        if (IsKeyPressed(KEY_SPACE))
        {
            isPaused = !isPaused;
        }
        
        if (IsKeyPressed(KEY_R))
        {
            stats = (Statistics){0};
            InitState(&components);
        }
        
        
        if (!isPaused)
        {
            for (int i = 0; i < (int)(speedMultiplier * 2); i++)
            {
                solvePendulum(&components, DT);
            }
            
            Vector2 updatedStartPos = {box.position.x + box.width / 2, box.position.y + box.height / 2};
            Vector2 end1 = get_end(&updatedStartPos, components.alpha1, components.l1);
            Vector2 end2 = get_end(&end1, components.alpha2, components.l2);
            updateStatistics(&stats, &components, &box.position, &box);
        }

        BeginDrawing();
        ClearBackground(BLACK);
        
        drawDoublePendulum(&box.position, &components, &box);
        drawMoveableBox(&box);
        DrawUI(&components, &stats);
        DrawFPS(20, HEIGHT - 50);
        
        EndDrawing();
    }
    
    CloseWindow();
    return 0;
}
