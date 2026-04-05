#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
//#include "i2c-lcd.h"


#define ROW 16
#define COL 16
#define INF 0x3f3f3f3f

// Directions: 0=N,1=E,2=S,3=W
static const int dx[4] = {0, 1, 0, -1};
static const int dy[4] = {1, 0, -1, 0};
static const int LMotorPin = 13, RMotorPin = 40, LInfrared = 12, RInfrared = 42, US_Echo = 46, US_Trig = 45;
static const char dirChar[4] = {'N','E','S','W'};

// walls bitmask per cell: bit0=N, bit1=E, bit2=S, bit3=W
static unsigned char walls[ROW][COL];
static bool known[ROW][COL];

static inline bool in_bounds(int x, int y){ return x >=0 && x < ROW && y>=0 && y < COL;}

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint32_t Is_First_Captured = 0;
uint32_t Distance = 0;




int API_wallFront() {
	US_Read();

}

void US_Read (void)
{
	HAL_GPIO_WritePin(US_Echo, US_Trig, 1);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(US_Echo, US_Trig, 0);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}

void delay (uint16_t time)
	{
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		while (__HAL_TIM_GET_COUNTER (&htim1) < time);
	}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}

void API_setWall(int x, int y, char direction) {
    printf("setWall %d %d %c\n", x, y, direction);
    fflush(stdout);
}

void dlog(char *text)
{
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}
static void mark_wall(int x, int y, int d){
    if(!in_bounds(x,y)) return;
    walls[x][y] |= (1<<d);
    int nx = x + dx[d];
    int ny = y + dy[d];
    if(in_bounds(nx,ny)){
        int od = (d+2)%4;
        walls[nx][ny] |= (1<<od);
    }
    API_setWall(x,y, dirChar[d]);
}

// Heuristic: Manhattan distance
static int heur(int x1,int y1,int x2,int y2){
    return abs(x1-x2) + abs(y1-y2);
}

// Simple binary heap open list for A*
typedef struct { int x,y; int g,h,f; int parent; } Node;

// A* returns path as sequence of coordinates in pathX[], pathY[], pathLen
static int a_star(int sx,int sy,int gx,int gy,int pathX[],int pathY[]){
    if(!in_bounds(sx,sy) || !in_bounds(gx,gy)) return 0;

    int maxn = ROW*COL;
    Node *nodes = (Node*)malloc(sizeof(Node)*maxn);
    bool closed[ROW][COL];
    int nodeCount = 0;
    memset(closed,0,sizeof(closed));

    // open list as simple array heap of indices into nodes
    int *heap = (int*)malloc(sizeof(int)*maxn);
    int heapSize = 0;

    // helper heap functions
    #define SWAP(i,j) { int t=heap[i]; heap[i]=heap[j]; heap[j]=t; }
    void heap_up(int idx){
        while(idx>0){
            int p=(idx-1)/2;
            if(nodes[heap[p]].f <= nodes[heap[idx]].f) break;
            SWAP(p,idx); idx=p;
        }
    }
    void heap_down(int idx){
        for(;;){
            int l=idx*2+1; if(l>=heapSize) break;
            int r=l+1; int s=l; if(r<heapSize && nodes[heap[r]].f < nodes[heap[l]].f) s=r;
            if(nodes[heap[s]].f >= nodes[heap[idx]].f) break;
            SWAP(s,idx); idx=s;
        }
    }

    // create start node
    nodes[nodeCount].x = sx; nodes[nodeCount].y = sy; nodes[nodeCount].g=0; nodes[nodeCount].h=heur(sx,sy,gx,gy);
    nodes[nodeCount].f = nodes[nodeCount].g + nodes[nodeCount].h; nodes[nodeCount].parent = -1;
    heap[heapSize++] = nodeCount;
    heap_up(heapSize-1);
    nodeCount++;

    while(heapSize>0){
        int curIdx = heap[0];
        // pop
        heap[0]=heap[--heapSize]; heap_down(0);
        int cx = nodes[curIdx].x, cy = nodes[curIdx].y;
        if(closed[cx][cy]) continue;
        closed[cx][cy]=true;
        if(cx==gx && cy==gy){
            // reconstruct path
            int plen = 0; int idx = curIdx;
            while(idx!=-1){ pathX[plen]=nodes[idx].x; pathY[plen]=nodes[idx].y; plen++; idx = nodes[idx].parent; }
            // reverse
            for(int i=0;i<plen/2;i++){ int tx=pathX[i], ty=pathY[i]; pathX[i]=pathX[plen-1-i]; pathY[i]=pathY[plen-1-i]; pathX[plen-1-i]=tx; pathY[plen-1-i]=ty; }
            free(nodes); free(heap);
            return plen;
        }

        // expand neighbors (N,E,S,W)
        for(int d=0;d<4;d++){
            int nx = cx + dx[d]; int ny = cy + dy[d];
            if(!in_bounds(nx,ny)) continue;
            // blocked by known wall?
            if(walls[cx][cy] & (1<<d)) continue;
            if(closed[nx][ny]) continue;
            // create node
            int gnew = nodes[curIdx].g + 1;
            // check if we already have a node for nx,ny with better g; naive linear search
            int found = -1;
            for(int i=0;i<nodeCount;i++) if(nodes[i].x==nx && nodes[i].y==ny){ found=i; break; }
            if(found!=-1){
                if(gnew < nodes[found].g){ nodes[found].g = gnew; nodes[found].f = gnew + nodes[found].h; nodes[found].parent = curIdx; }
                // push/update on heap
                heap[heapSize++] = found; heap_up(heapSize-1);
            } else {
                nodes[nodeCount].x = nx; nodes[nodeCount].y = ny; nodes[nodeCount].g = gnew; nodes[nodeCount].h = heur(nx,ny,gx,gy);
                nodes[nodeCount].f = nodes[nodeCount].g + nodes[nodeCount].h; nodes[nodeCount].parent = curIdx;
                heap[heapSize++] = nodeCount; heap_up(heapSize-1);
                nodeCount++;
            }
        }
    }

    free(nodes);
    free(heap);
    return 0; // no path
}

// Map sensing: rel -1 left, 0 front, +1 right
static int rel_to_abs(int facing, int rel){
    if(rel==0)
        return facing;
    if(rel==-1)
        return (facing+3)%4;
    if(rel==1)
        return (facing+1)%4;
    return -1;
}

void astar_init(void){
    memset(walls,0,sizeof(walls));
    memset(known,0,sizeof(known));
}

int main(int argc, char *argv[]){
    int x=0,y=0; int facing=0; // start (0,0) facing north
    API_setColor(x,y,'G'); API_setText(x,y,"start");

    // loop until reach center
    while(true){
        // sense walls and mark
        if(API_wallFront()){ int d=rel_to_abs(facing,0); mark_wall(x,y,d); }
        if(API_wallLeft()){ int d=rel_to_abs(facing,-1); mark_wall(x,y,d); }
        if(API_wallRight()){ int d=rel_to_abs(facing,1); mark_wall(x,y,d); }
        known[x][y]=true;

        // if in center zone, done
        if((x==7 || x==8) && (y==7 || y==8)) { API_setColor(x,y,'B'); API_setText(x,y,"goal"); break; }

        // plan path to nearest center (try 4 center cells, pick shortest A* path length)
        int bestLen=INF; int bestGX=7,bestGY=7; int tmpPathX[ROW*COL], tmpPathY[ROW*COL];
        int centers[4][2] = {{7,7},{7,8},{8,7},{8,8}};
        int bestPathX[ROW*COL], bestPathY[ROW*COL], bestPLen=0;
        for(int c=0;c<4;c++){
            int gx=centers[c][0], gy=centers[c][1];
            int len = a_star(x,y,gx,gy,tmpPathX,tmpPathY);
            if(len>0 && len < bestLen){ bestLen=len; bestGX=gx; bestGY=gy; bestPLen=len; memcpy(bestPathX,tmpPathX,sizeof(int)*len); memcpy(bestPathY,tmpPathY,sizeof(int)*len); }
        }

        if(bestLen==INF){
            // no known path; naive exploration: try to go forward if no wall, else turn right
            if(!API_wallFront()){ API_moveForward(); x += dx[facing]; y += dy[facing]; API_setColor(x,y,'Y'); continue; }
            else if(!API_wallRight()){ API_turnRight(); facing = (facing+1)%4; continue; }
        }

        // follow planned path step-by-step; replan if blocked
        for(int step=1; step<bestPLen; step++){
            int nx = bestPathX[step]; int ny = bestPathY[step];
            // determine required absolute direction
            int neededDir = -1; for(int d=0;d<4;d++){ if(x+dx[d]==nx && y+dy[d]==ny) { neededDir=d; break; } }
            if(neededDir==-1) { break; }

            // Read sensors ONCE at current pose and reuse results to decide turning/moving.
            bool sensedFront = API_wallFront(); if(sensedFront){ int d=rel_to_abs(facing,0); mark_wall(x,y,d); }
            bool sensedLeft  = API_wallLeft();  if(sensedLeft) { int d=rel_to_abs(facing,-1); mark_wall(x,y,d); }
            bool sensedRight = API_wallRight(); if(sensedRight){ int d=rel_to_abs(facing,1); mark_wall(x,y,d); }

            // turn to neededDir
            int turn = (neededDir - facing + 4) % 4;
            if(turn==1){
                // turning right: check the previously read right sensor instead of calling again
                if (!sensedRight) { API_turnRight(); facing = neededDir; }
                else { int d=rel_to_abs(facing,1); mark_wall(x,y,d); break; }
            }
            else if(turn==3){
                // turning left: check the previously read left sensor
                if (!sensedLeft) { API_turnLeft(); facing = neededDir; }
                else { int d=rel_to_abs(facing,-1); mark_wall(x,y,d); break; }
            }
            else if(turn==2){
                // about-face: we didn't sense the back; perform the two turns
                API_turnRight(); API_turnRight(); facing = neededDir;
            }

            // map previously read sensors to the new front sensor where possible
            bool frontBlockedAfterTurn = false;
            if(turn==0) frontBlockedAfterTurn = sensedFront;
            else if(turn==1) frontBlockedAfterTurn = sensedRight;
            else if(turn==3) frontBlockedAfterTurn = sensedLeft;
            else frontBlockedAfterTurn = API_wallFront(); // for about-face, query front now

            if(frontBlockedAfterTurn){ // unexpected wall (or discovered via mapping)
                int wd = rel_to_abs(facing,0); mark_wall(x,y,wd);
                // replan
                break;
            }

            // move forward
            API_moveForward();
            x = nx; y = ny;
            API_setColor(x,y,'Y');

            // sense and mark newly discovered walls at new position
            if(API_wallFront()){ int d=rel_to_abs(facing,0); mark_wall(x,y,d); }
            if(API_wallLeft()){ int d=rel_to_abs(facing,-1); mark_wall(x,y,d); }
            if(API_wallRight()){ int d=rel_to_abs(facing,1); mark_wall(x,y,d); }

            // if we reached center, exit
            if((x==7 || x==8) && (y==7 || y==8)) { API_setColor(x,y,'B'); API_setText(x,y,"goal"); return; }
        }
        // loop will replan
    }
}
