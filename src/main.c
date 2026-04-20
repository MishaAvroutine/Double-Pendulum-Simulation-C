#include<stdio.h>
#include"../include/raylib.h"
#include"../include/raymath.h"
#include<math.h>


#define HEIGHT 600
#define WIDTH 800


#define THICKNESS_OF_LINE 10
#define MASS_RADIUS 20


#define L1 250.0
#define L2 200.0
#define DT 0.01

#define G 100

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


void drawDoublePendulum(Vector2* startPos,PendulumComponents* components)
{
    Vector2 end_first = get_end(startPos,components->alpha1,components->l1);
    drawPendulum(&end_first,components->alpha2,components->l2);
    drawPendulum(startPos,components->alpha1,components->l1);
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
    components->alpha1_dd = (-G / components->l1) * sinf(components->alpha1);

    components->alpha1_d += components->alpha1_dd * dt;

    components->alpha1 += components->alpha1_d * dt;


    components->alpha2_dd = (-G / components->l2) * sinf(components->alpha2);

    components->alpha2_d += components->alpha2_dd * dt;

    components->alpha2 += components->alpha2_d * dt;
}



int main()
{
    // init window
    InitWindow(WIDTH,HEIGHT,"Double Pendulum");

    // init components
    PendulumComponents components= {0};
    InitState(&components);

    // start pos.
    Vector2 startPos = {WIDTH / 2, 0};

    SetTargetFPS(120);
    // main loop
    while(!WindowShouldClose())
    {
        solvePendulum(&components,DT);
        BeginDrawing();
        ClearBackground(BLACK);
        DrawFPS(20,20);
        drawDoublePendulum(&startPos,&components);
        EndDrawing();
    }
    return 0;
}
