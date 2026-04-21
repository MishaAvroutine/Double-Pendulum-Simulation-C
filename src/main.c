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
    components->alpha1 = GetRandomValue(-90, 90) * DEG2RAD;
    components->alpha1_d = 0;
    components->alpha1_dd = 0;
    components->l1 = L1;

    components->m2 = 20.0;
    components->alpha2 = GetRandomValue(-90, 90) * DEG2RAD;
    components->alpha2_d = 0;
    components->alpha2_dd = 0;
    components->l2 = L2;
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
        components->alpha1 *= DAMPEN;
        components->alpha2 *= DAMPEN;
}


void DrawUI(PendulumComponents* components)
{
    DrawText("G: ",0,HEIGHT - MARGIN,FONT_SIZE,BLUE);
    GuiSliderBar((Rectangle){SLIDER_BAR_X_MARGIN,HEIGHT - MARGIN,SLIDER_BAR_WIDTH,SLIDER_BAR_HEIGHT},"0.0","1000.0",&G,MIN_G,MAX_G);

    DrawText("L1: ",0,HEIGHT- MARGIN * 5,FONT_SIZE,BLUE);
    GuiSliderBar((Rectangle){SLIDER_BAR_X_MARGIN + 10,HEIGHT - MARGIN*5,SLIDER_BAR_WIDTH,SLIDER_BAR_HEIGHT},"0.0","400.0",&components->l1,MIN_L,MAX_L);

    DrawText("L2: ",0,HEIGHT- MARGIN * 4,FONT_SIZE,BLUE);
    GuiSliderBar((Rectangle){SLIDER_BAR_X_MARGIN + 10,HEIGHT - MARGIN*4,SLIDER_BAR_WIDTH,SLIDER_BAR_HEIGHT},"0.0","400.0",&components->l2,MIN_L,MAX_L);

    DrawText("M1: ",0,HEIGHT- MARGIN * 3,FONT_SIZE,BLUE);
    GuiSliderBar((Rectangle){SLIDER_BAR_X_MARGIN + 10,HEIGHT - MARGIN*3,SLIDER_BAR_WIDTH,SLIDER_BAR_HEIGHT},"0.0","100.0",&components->m2,MIN_MASS,MAX_MASS);

    DrawText("M2: ",0,HEIGHT- MARGIN * 2,FONT_SIZE,BLUE);
    GuiSliderBar((Rectangle){SLIDER_BAR_X_MARGIN + 10,HEIGHT - MARGIN*2,SLIDER_BAR_WIDTH,SLIDER_BAR_HEIGHT},"0.0","100.0",&components->m1,MIN_MASS,MAX_MASS);

    DrawText("Damp: ",0,HEIGHT - MARGIN * 6,FONT_SIZE,BLUE);
    GuiCheckBox((Rectangle){SLIDER_BAR_X_MARGIN + 10,HEIGHT - MARGIN*6,30,25},dampen ? "True" : "False",&dampen);


}



int main()
{
    // init window
    InitWindow(WIDTH,HEIGHT,"Double Pendulum");
    GuiLoadStyleJungle();
    // init components
    PendulumComponents components= {0};
    InitState(&components);

    MoveableBox box = {0};
    box.position = (Vector2){WIDTH / 2, 0};
    box.width = 30;
    box.height = 30;
    box.isDragging = false;

    SetTargetFPS(120);
    // main loop
    while(!WindowShouldClose())
    {
        updateMoveableBox(&box);
        solvePendulum(&components,DT);

        if(IsKeyPressed(KEY_R))
        {
            InitState(&components);
        }

        BeginDrawing();
        ClearBackground(BLACK);
        DrawUI(&components);
        DrawFPS(20,20);
        drawDoublePendulum(&box.position,&components,&box);
        drawMoveableBox(&box);
        EndDrawing();
    }
    return 0;
}
