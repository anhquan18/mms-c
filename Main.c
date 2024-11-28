#include <stdio.h>
#include "API.h"


#define MAZESIZE_X	(16)		//$BLBO)$NBg$-$5(B(MAZESIZE_X * MAZESIZE_Y)$BLBO)(B
#define MAZESIZE_Y	(16)		//$BLBO)$NBg$-$5(B(MAZESIZE_X * MAZESIZE_Y)$BLBO)(B
#define MAX_X	(MAZESIZE_X*2)		//$BLBO)$NBg$-$5(B(MAZESIZE_X * MAZESIZE_Y)$BLBO)(B
#define MAX_Y	(MAZESIZE_Y*2)		//$BLBO)$NBg$-$5(B(MAZESIZE_X * MAZESIZE_Y)$BLBO)(B
#define GOAL_X	7							//$B%4!<%k:BI8(B(x)
#define GOAL_Y	7							//$B%4!<%k:BI8(B(y)

#define PI (3.141592653589793)	//$B1_<~N((B

#define DIAGONAL_QUARTER_SECTION (63.639)	//1/4$B6h2h$N<P$a5wN%(B
#define QUARTER_SECTION	(45.0)	//1/4$B6h2h$N5wN%(B
#define HALF_SECTION	(90.0)	//$BH>6h2h$N5wN%(B
#define SECTION		(180.0)		//$B0l6h2h$N5wN%(B

#define UNKNOWN	2				//$BJI$,$"$k$+$J$$$+H=$i$J$$>uBV$N>l9g$NCM(B
#define NOWALL	0				//$BJI$,$J$$$P$"$$$NCM(B
#define WALL	1				//$BJI$,$"$k>l9g$NCM(B
#define VWALL	3				//$B2>A[JI$NCM(B($BL$;HMQ(B)

#define MASK_SEARCH	0x01		//$BC5:wAv9TMQ%^%9%/CM(B.$BJI>pJs$H$3$NCM$N(BAND$BCM$,#0!J(BNOWALL$B!K$J$iJI$J$7(Bor$BL$C5:w6h4V(B
#define MASK_SECOND	0x03		//$B:GC;Av9TMQ%^%9%/CM(B.$BJI>pJs$H$3$NCM$N(BAND$BCM$,#0!J(BNOWALL$B!K$J$iJI$J$7(B


// turn 90 time
#define DAIKEI_ZENSHIN (500.0)
#define DAIKEI_END_TIME (299)
#define ZENSHIN_MAX_SPEED (1100.0)

// straight end time
#define ZENSHIN_END_TIME (HALF_SECTION/DAIKEI_ZENSHIN)
//
// straight end time
#define NANAME_END_TIME (DIAGONAL_QUARTER_SECTION/DAIKEI_ZENSHIN)

// turn 180 time
#define DAIKEI_45IN_END_TIME (212)
#define DAIKEI_45OUT_END_TIME (232)
#define DAIKEI_V90_END_TIME (429)
#define DAIKEI_135IN_END_TIME (474)
#define DAIKEI_135OUT_END_TIME (474)
#define DAIKEI_OOMAWARI180_END_TIME (570)

//$BCO?^$N8~$-(B
#define M_FRONT (0)		//$BA0(B
#define M_RIGHT (1)		//$B1&(B
#define M_REAR (2)			//$B8e(B
#define M_LEFT (3)			//$B:8(B
#define M_RIGHT_N (19)		//$B1&(B90$BBg2s$j(B
#define M_LEFT_N (20)		//$B:8(B90$BBg2s$j(B
#define M_RIGHT_F (17)		//$B1&(B90$BBg2s$j(B
#define M_LEFT_F (18)		//$B:8(B90$BBg2s$j(B
#define M_RIGHT45 (4)		//$B1&(B45
#define M_LEFT45 (5)		//$B:8(B45
#define M_RIGHTV90 (6)		//$B1&(BV90
#define M_LEFTV90 (7)		//$B:8(BV90
#define M_RIGHT135 (8)		//$B1&(B135
#define M_LEFT135 (9)		//$B:8(B135
#define M_RIGHT180 (10)	        //$B1&(B180
#define M_LEFT180 (11)		//$B:8(B180
#define M_RIGHT180_F (15)       //$B1&(B180$BBg2s$j(B
#define M_LEFT180_F (16)	//$B:8(B180$BBg2s$j(B
#define M_RIGHT45OUT (12)	//$B1&(B45$B=P8}(B
#define M_LEFT45OUT (13)	//$B:8(B45$B=P8}(B
#define M_FRONT_DIAGONAL (14) //$B<P$aA0?J(B



typedef enum
{
	false = 0,	//$B56(B
	true = 1,	//$B??(B
}t_bool;		//$B??56CM$r<h$j07$&Ns5s7?(B

typedef enum
{
	front=0,		//$BA0(B
	right=1,		//$B1&(B
	rear=2,			//$B8e(B
	left=3,			//$B:8(B
	stop=5,
	unknown,		//$BJ}8~ITL@(B
}t_local_dir;	//$B<+J,$+$i8+$?J}8~$r<($9Ns5s7?(B

typedef enum
{
	north=0,
	east=1,
	south=2,
	west=3,
	north_east=4,
	north_west=5,
	south_east=6,
	south_west=7,
}t_direction;

typedef struct
{
	int x;
	int y;
	t_direction dir;
}t_position;

typedef struct
{
	unsigned char north:2;	//$BKL$NJI>pJs(B
	unsigned char east:2;	//$BEl$NJI>pJs(B
	unsigned char south:2;	//$BFn$NJI>pJs(B
	unsigned char west:2;	//$B@>$NJI>pJs(B
}t_wall;					//$BJI>pJs$r3JG<$9$k9=B$BN(B($B%S%C%H%U%#!<%k%I(B)



short already_passed_boolean_map[256] = {0};
short unable_to_find_path_to_goal = 0;
t_position		mypos;							//$B<+8J:BI8(B
t_wall			wall[MAZESIZE_X][MAZESIZE_Y];	//$BJI$N>pJs$r3JG<$9$k9=B$BNG[Ns(B
unsigned char  wall_naname[MAZESIZE_X*2+1][MAZESIZE_Y*2+1];	//$BJI$N>pJs$r3JG<$9$k9=B$BNG[Ns(B
unsigned char	map[MAZESIZE_X][MAZESIZE_Y];	//$BJb?t%^%C%W(B
unsigned short	map_naname[MAZESIZE_X*2+1][MAZESIZE_Y*2+1];	//$BJb?t%^%C%W(B
short shortest_route_action[256]; // $B?J$`!&:8!&1&$J$I$N8~$-$GA0?J!&2sE>$rI=8=(B
float shortest_route_action_times[256]; // $B?t;z$GEv$F$O$^$k9TF0(B($BA0?J!&2sE>(B)$B$N2s?t$rI=8=(B
short diagonal_route_action[256]; // $B?J$`!&:8!&1&$J$I$N8~$-$GA0?J!&2sE>$rI=8=(B
float diagonal_route_action_times[256]; // $B?t;z$GEv$F$O$^$k<P$a9TF0(B($BA0?J!&2sE>(B)$B$N2s?t$rI=8=(B
unsigned short fast_straight_cost = (unsigned short) (ZENSHIN_END_TIME*1000);
unsigned short fast_naname_cost = (unsigned short) (NANAME_END_TIME*1000);
char color = 'G';
unsigned short recusion_weight = 0;



void init_map(int x, int y)
{
//$BLBO)$NJb?t(BMap$B$r=i4|2=$9$k!#A4BN$r(B0xff$B!"0z?t$N:BI8(Bx,y$B$O(B0$B$G=i4|2=$9$k(B
	int i,j;
	for(i = 0; i < MAZESIZE_X; i++)		//$BLBO)$NBg$-$5J,%k!<%W(B(x$B:BI8(B)
	{
		for(j = 0; j < MAZESIZE_Y; j++)	//$BLBO)$NBg$-$5J,%k!<%W(B(y$B:BI8(B)
		{
			map[i][j] = 255;			//$B$9$Y$F(B255$B$GKd$a$k(B
            API_setText(i, j, "255");
		}
	}
	map[x][y] = 0;						//$B%4!<%k:BI8$NJb?t$r#0$K@_Dj(B
    API_setText(x, y, "0");
}


void init_map_naname(int x, int y)
{
//$BLBO)$NJb?t(BMap$B$r=i4|2=$9$k!#A4BN$r(B0xff$B!"0z?t$N:BI8(Bx,y$B$O(B0$B$G=i4|2=$9$k(B
	int i,j;
	for(i = 0; i < MAZESIZE_X*2+1; i++)		//$BLBO)$NBg$-$5J,%k!<%W(B(x$B:BI8(B)
	{
		for(j = 0; j < MAZESIZE_Y*2+1; j++)	//$BLBO)$NBg$-$5J,%k!<%W(B(y$B:BI8(B)
		{
			map_naname[i][j] = 59999;			//$B$9$Y$F(B999$B$GKd$a$k(B
		}
	}
	map_naname[x*2+1][y*2+1] = 0;						//$B%4!<%k:BI8$NJb?t$r#0$K@_Dj(B
}



void make_map(int x, int y, int mask)	//$BJb?t%^%C%W$r:n@.$9$k(B
{
//$B:BI8(Bx,y$B$r%4!<%k$H$7$?Jb?t(BMap$B$r:n@.$9$k!#(B
//mask$B$NCM(B(MASK_SEARCH or MASK_SECOND)$B$K$h$C$F!"(B
//$BC5:wMQ$NJb?t(BMap$B$r:n$k$+!":GC;Av9T$NJb?t(BMap$B$r:n$k$+$,@Z$jBX$o$k(J\(B

	int i,j;
	t_bool change_flag;										//Map$B:n@.=*N;$r8+6K$a$k$?$a$N%U%i%0(B
    char str[5];

    API_clearAllText();
	init_map(x,y);											//Map$B$r=i4|2=$9$k(B
	do
	{
		change_flag = false;								//$BJQ99$,$J$+$C$?>l9g$K$O%k!<%W$rH4$1$k(B
		for(i = 0; i < MAZESIZE_X; i++)						//$BLBO)$NBg$-$5J,%k!<%W(B(x$B:BI8(B)
		{
			for(j = 0; j < MAZESIZE_Y; j++)					//$BLBO)$NBg$-$5J,%k!<%W(B(y$B:BI8(B)
			{
				if(map[i][j] == 255)						//255$B$N>l9g$O<!$X(B
				{
					continue;
				}
				
				if(j < MAZESIZE_Y-1)						//$BHO0O%A%'%C%/(B
				{
					if( (wall[i][j].north & mask) == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
					{
						if(map[i][j+1] == 255)				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
						{
							map[i][j+1] = map[i][j] + 1;	//$BCM$rBeF~(B
                            sprintf(str, "%u", map[i][j+1]);
                            API_setText(i, j+1, str);
							change_flag = true;				//$BCM$,99?7$5$l$?$3$H$r<($9(B
						}
					}
				}
			
				if(i < MAZESIZE_X-1)						//$BHO0O%A%'%C%/(B
				{
					if( (wall[i][j].east & mask) == NOWALL)	//$BJI$,$J$1$l$P(B
					{
						if(map[i+1][j] == 255)				//$BCM$,F~$C$F$$$J$1$l$P(B
						{
							map[i+1][j] = map[i][j] + 1;	//$BCM$rBeF~(B
                            sprintf(str, "%u", map[i+1][j]);
                            API_setText(i+1, j, str);
							change_flag = true;				//$BCM$,99?7$5$l$?$3$H$r<($9(B
						}
					}
				}
			
				if(j > 0)									//$BHO0O%A%'%C%/(B
				{
					if( (wall[i][j].south & mask) == NOWALL)//$BJI$,$J$1$l$P(B
					{
						if(map[i][j-1] == 255)				//$BCM$,F~$C$F$$$J$1$l$P(B
						{
							map[i][j-1] = map[i][j] + 1;	//$BCM$rBeF~(B
                            sprintf(str, "%u", map[i][j-1]);
                            API_setText(i, j-1, str);
							change_flag = true;				//$BCM$,99?7$5$l$?$3$H$r<($9(B
						}
					}
				}
			
				if(i > 0)									//$BHO0O%A%'%C%/(B
				{
					if( (wall[i][j].west & mask) == NOWALL)	//$BJI$,$J$1$l$P(B
					{
						if(map[i-1][j] == 255)				//$BCM$,F~$C$F$$$J$1$l$P(B
						{
							map[i-1][j] = map[i][j] + 1;	//$BCM$rBeF~(B	
                            sprintf(str, "%u", map[i-1][j]);
                            API_setText(i-1, j, str);
							change_flag = true;				//$BCM$,99?7$5$l$?$3$H$r<($9(B
						}
						
					}
					
				}
				
			}
			
		}
		
	}while(change_flag == true);	//$BA4BN$r:n$j=*$o$k$^$GBT$D(B
}



void check_and_set_wall(int x, int y, int is_wall, char direction)
{
    if(is_wall) {
        API_setWall(x,y,direction);
    } 
}


void set_wall(int x, int y)	//$BJI>pJs$r5-O?(B
{
//$B0z?t$N:BI8(Bx,y$B$KJI>pJs$r=q$-9~$`(B
	int n_write,s_write,e_write,w_write;
	
	
	//$B<+J,$NJ}8~$K1~$8$F=q$-9~$`%G!<%?$r@8@.(B
	//CONV_SEN2WALL()$B$O(Bmacro.h$B$r;2>H(B
	switch(mypos.dir){
		case north:	//$BKL$r8~$$$F$$$k;~(B
			n_write = API_wallFront();	//$BA0JI$NM-L5$rH=CG(B
			e_write = API_wallRight();  //$B1&JI$NM-L5$rH=CG(B
			w_write = API_wallLeft();   //$B:8JI$NM-L5$rH=CG(B
			s_write = NOWALL;		    //$B8e$m$OI,$:JI$,$J$$(B
			break;
			
		case east:	//$BEl$r8~$$$F$$$k$H$-(B
			e_write = API_wallFront();	//$BA0JI$NM-L5$rH=CG(B
			s_write = API_wallRight();  //$B1&JI$NM-L5$rH=CG(B
			n_write = API_wallLeft();   //$B:8JI$NM-L5$rH=CG(B
			w_write = NOWALL;											//$B8e$m$OI,$:JI$,$J$$(B
			break;
			
		case south:	//$BFn$r8~$$$F$$$k$H$-(B
			s_write = API_wallFront();	//$BA0JI$NM-L5$rH=CG(B
			w_write = API_wallRight();  //$B1&JI$NM-L5$rH=CG(B
			e_write = API_wallLeft();   //$B:8JI$NM-L5$rH=CG(B
			n_write = NOWALL;											//$B8e$m$OI,$:JI$,$J$$(B
			break;
			
		case west:	//$B@>$r8~$$$F$$$k$H$-(B
			w_write = API_wallFront();		//$BA0JI$NM-L5$rH=CG(B
			n_write = API_wallRight();		//$B1&JI$NM-L5$rH=CG(B
			s_write = API_wallLeft();		//$B:8JI$NM-L5$rH=CG(B
			e_write = NOWALL;			NOWALL;											//$B8e$m$OI,$:JI$,$J$$(B
			break;
	}
	
	wall[x][y].north = n_write;	//$B<B:]$KJI>pJs$r=q$-9~$_(B
    check_and_set_wall(x,y,n_write,'n');	

    wall[x][y].south = s_write;	//$B<B:]$KJI>pJs$r=q$-9~$_(B
    check_and_set_wall(x,y,s_write,'s');	

	wall[x][y].east  = e_write;	//$B<B:]$KJI>pJs$r=q$-9~$_(B
    check_and_set_wall(x,y,e_write,'e');	

	wall[x][y].west  = w_write;	//$B<B:]$KJI>pJs$r=q$-9~$_(B
    check_and_set_wall(x,y,w_write,'w');	
	
	if(y < MAZESIZE_Y-1)				//$BHO0O%A%'%C%/(B
	{
		wall[x][y+1].south = n_write;	//$BH?BPB&$+$i8+$?JI$r=q$-9~$_(B
        //check_and_set_wall(x, y+1, n_write,'s');	
	}
	
	if(x < MAZESIZE_X-1)				//$BHO0O%A%'%C%/(B
	{
		wall[x+1][y].west = e_write;	//$BH?BPB&$+$i8+$?JI$r=q$-9~$_(B
        //check_and_set_wall(x+1, y, e_write,'w');	
	}
	
	if(y > 0)							//$BHO0O%A%'%C%/(B
	{
		wall[x][y-1].north = s_write;	//$BH?BPB&$+$i8+$?JI$r=q$-9~$_(B
        //check_and_set_wall(x, y-1, s_write,'n');	
	}
	
	if(x > 0)							//$BHO0O%A%'%C%/(B
	{
		wall[x-1][y].east = w_write;	//$BH?BPB&$+$i8+$?JI$r=q$-9~$_(B
        //check_and_set_wall(x-1, y, w_write,'e');	
	}
	
}



void set_wall_whole_map(void) {
    int x, y;
    for (x=0; x<MAZESIZE_X; x++) {
        for (y=0; y<MAZESIZE_Y; y++) {
            mypos.x = x;
            mypos.y = y;
            mypos.dir = north;
            set_wall(x,y);
            API_setColor(x,y,'A');

            API_turnLeft();
            API_turnLeft();
            mypos.dir = south;
            set_wall(x,y);

            API_turnLeft();
            API_turnLeft();
            mypos.dir = north;
            API_moveForward();
        }
        if ((y == MAZESIZE_Y) && (x == MAZESIZE_X-1)) {
            API_turnLeft();
            for(int i =0; i<MAZESIZE_Y-1; i++) {
                API_moveForward();
            }
            API_turnLeft();
            for(int i =0; i<MAZESIZE_Y; i++) {
                API_moveForward();
            }
            API_turnLeft();
            API_turnLeft();
            continue;
        } 
        if(y == MAZESIZE_Y){
            API_turnRight();
            API_moveForward();
            API_turnRight();
            for(int i =0; i<MAZESIZE_Y; i++) {
                API_moveForward();
            }
            mypos.x = x;
            mypos.y = 0;
            mypos.dir = south;
            set_wall(x,0);
            API_setColor(x,y,'A');
            API_turnRight();
            API_turnRight();
            continue;
        } 
    }
}



void prioritize_straight_cost_recursion(short x, short y, t_direction prev_dir, t_direction current_dir, short weight) {
    //fprintf(stderr, "\nx: %d, y: %d\n", x, y);
    //fflush(stderr);
    weight -= recusion_weight;
    if (weight <= 0){
        weight = 30;
    }
    switch(current_dir) {
        case north:
            if(y < MAX_Y)	// $BA0?JF0:n(B
            {
            	if (wall_naname[x][y+1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    if(prev_dir != current_dir) { // $B8~$-$,JQ$o$C$?>l9g!"2sE>$N=E$_$+$iD>@~$N=E$_$K%j%;%C%H(B
                        weight = fast_straight_cost;
                    }
            		if((map_naname[x][y] + weight) <= map_naname[x][y+1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x][y+1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x, y+1, north, north, weight);
                    }
            	}
            }

            if((y < MAX_Y) && (x < MAX_X))	// $B1&(B45$BEY2sE>(B
            {
            	if (wall_naname[x+1][y+1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_45IN_END_TIME; // $B8~$-$r(B45$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y+1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x+1][y+1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x+1, y+1, north, north_east, weight);
                    }
            	}
            }

            if((y < MAX_Y) && (x > 0))	// $B:8(B45$BEY2sE>(B
            {
            	if (wall_naname[x-1][y+1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_45IN_END_TIME; // $B8~$-$r(B45$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y+1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x-1][y+1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x-1, y+1, north, north_west, weight);
                    }
            	}
            }

            if(x < MAX_X)	// $B1&(B90$BEY2sE>(B
            {
            	if (wall_naname[x+1][y] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_END_TIME; // $B8~$-$r(B90$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x+1][y] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x+1, y, north, east, weight);
                    }
            	}
            }

            if(x > 0)	// $B:8(B90$BEY2sE>(B
            {
            	if (wall_naname[x-1][y] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_END_TIME; // $B8~$-$r(B90$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x-1][y] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x-1, y, north, west, weight);
                    }
            	}
            }

            break;

        case north_east:
            if((y < MAX_Y) && (x < MAX_X))						//$BHO0O%A%'%C%/(B
            {
            	if (wall_naname[x+1][y+1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    if(prev_dir != current_dir) { // $B8~$-$,JQ$o$C$?>l9g!"2sE>$N=E$_$+$iD>@~$N=E$_$K%j%;%C%H(B
                        weight = fast_naname_cost;
                    }
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y+1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x+1][y+1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x+1, y+1, north_east, north_east, weight);
                    }
            	}
            }
            if(x < MAX_X)	// $B1&(B45$BEY2sE>(B
            {
            	if (wall_naname[x+1][y] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_45OUT_END_TIME; // $B8~$-$r(B45$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x+1][y] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x+1, y, north_east, east, weight);
                    }
            	}
            }

            if(y < MAX_Y)	// $B:8(B45$BEY2sE>(B
            {
            	if (wall_naname[x][y+1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_45OUT_END_TIME; // $B8~$-$r(B45$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x][y+1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x][y+1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x, y+1, north_east, north, weight);
                    }
            	}
            }

            if((x < MAX_X) && (y > 0))	// $B1&(B90$BEY2sE>(B
            {
            	if (wall_naname[x+1][y-1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_END_TIME; // $B8~$-$r(B90$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y-1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x+1][y-1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x+1, y-1, north_east, south_east, weight);
                    }
            	}
            }

            if((x > 0) && (y < MAX_Y))	// $B:8(B90$BEY2sE>(B
            {
            	if (wall_naname[x-1][y+1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_END_TIME; // $B8~$-$r(B90$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y+1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x-1][y+1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x-1, y+1, north_east, north_west, weight);
                    }
            	}
            }
            break;

        case north_west:
            if((y < MAX_Y) && (x > 0))						//$BHO0O%A%'%C%/(B
            {
            	if (wall_naname[x-1][y+1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    if(prev_dir != current_dir) { // $B8~$-$,JQ$o$C$?>l9g!"2sE>$N=E$_$+$iD>@~$N=E$_$K%j%;%C%H(B
                        weight = fast_naname_cost;
                    }
            		if ((map_naname[x][y] + weight) <= map_naname[x-1][y+1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x-1][y+1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x-1, y+1, north_west, north_west, weight);
                    }
            	}
            }

            if(y < MAX_Y)	// $B1&(B45$BEY2sE>(B
            {
            	if (wall_naname[x][y+1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_45OUT_END_TIME; // $B8~$-$r(B45$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x][y+1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x][y+1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x, y+1, north_west, north, weight);
                    }
            	}
            }

            if(x > 0)	// $B:8(B45$BEY2sE>(B
            {
            	if (wall_naname[x-1][y] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_45OUT_END_TIME; // $B8~$-$r(B45$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x-1][y] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x-1, y, north_west, west, weight);
                    }
            	}
            }

            if((x < MAX_X) && (y < MAX_Y))	// $B1&(B90$BEY2sE>(B
            {
            	if (wall_naname[x+1][y+1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_END_TIME; // $B8~$-$r(B90$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y+1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x+1][y+1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x+1, y+1, north_west, north_east, weight);
                    }
            	}
            }

            if((x > 0) && (y > 0))	// $B:8(B90$BEY2sE>(B
            {
            	if (wall_naname[x-1][y-1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_END_TIME; // $B8~$-$r(B90$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y-1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x-1][y-1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x-1, y-1, north_west, south_west, weight);
                    }
            	}
            }
            break;

        case south:
            if(y > 0) 						//$BHO0O%A%'%C%/(B
            {
            	if (wall_naname[x][y-1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    if(prev_dir != current_dir) { // $B8~$-$,JQ$o$C$?>l9g!"2sE>$N=E$_$+$iD>@~$N=E$_$K%j%;%C%H(B
                        weight = fast_straight_cost;
                    }
            		if ((map_naname[x][y] + weight) <= map_naname[x][y-1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x][y-1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x, y-1, south, south, weight);
                    }
            	}
            }

            if((y > 0) && (x > 0))	// $B1&(B45$BEY2sE>(B
            {
            	if (wall_naname[x-1][y-1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_45IN_END_TIME; // $B8~$-$r(B45$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y-1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x-1][y-1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x-1, y-1, south, south_west, weight);
                    }
            	}
            }

            if((y > 0) && (x < MAX_X))	// $B:8(B45$BEY2sE>(B
            {
            	if (wall_naname[x+1][y-1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_45IN_END_TIME; // $B8~$-$r(B45$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y-1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x+1][y-1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x+1, y-1, south, south_east, weight);
                    }
            	}
            }

            if(x > 0)	// $B1&(B90$BEY2sE>(B
            {
            	if (wall_naname[x-1][y] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_END_TIME; // $B8~$-$r(B90$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x-1][y] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x-1, y, south, west, weight);
                    }
            	}
            }

            if(x < MAX_X)	// $B:8(B90$BEY2sE>(B
            {
            	if (wall_naname[x+1][y] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_END_TIME; // $B8~$-$r(B90$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x+1][y] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x+1, y, south, east, weight);
                    }
            	}
            }
            break;

        case south_east:
            if((y > 0) && (x < MAX_X))						//$BHO0O%A%'%C%/(B
            {
            	if (wall_naname[x+1][y-1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    if(prev_dir != current_dir) { // $B8~$-$,JQ$o$C$?>l9g!"2sE>$N=E$_$+$iD>@~$N=E$_$K%j%;%C%H(B
                        weight = fast_naname_cost;
                    }
            		if ((map_naname[x][y] + weight) <= map_naname[x+1][y-1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x+1][y-1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x+1, y-1, south_east, south_east, weight);
                    }
            	}
            }

            if(y > 0)	// $B1&(B45$BEY2sE>(B
            {
            	if (wall_naname[x][y-1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_45OUT_END_TIME; // $B8~$-$r(B45$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x][y-1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x][y-1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x, y-1, south_east, south, weight);
                    }
            	}
            }

            if(x < MAX_X)	// $B:8(B45$BEY2sE>(B
            {
            	if (wall_naname[x+1][y] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_45OUT_END_TIME; // $B8~$-$r(B45$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x+1][y] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x+1, y, south_east, east, weight);
                    }
            	}
            }

            if((x > 0) && (y > 0))	// $B1&(B90$BEY2sE>(B
            {
            	if (wall_naname[x-1][y-1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_END_TIME; // $B8~$-$r(B90$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y-1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x-1][y-1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x-1, y-1, south_east, south_west, weight);
                    }
            	}
            }

            if((x < MAX_X) && (y < MAX_Y))	// $B:8(B90$BEY2sE>(B
            {
            	if (wall_naname[x+1][y+1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_END_TIME; // $B8~$-$r(B90$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y+1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x+1][y+1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x+1, y+1, south_east, north_east, weight);
                    }
            	}
            }
            break;

        case south_west:
            if((y > 0) && (x > 0))						//$BHO0O%A%'%C%/(B
            {
            	if (wall_naname[x-1][y-1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    if(prev_dir != current_dir) { // $B8~$-$,JQ$o$C$?>l9g!"2sE>$N=E$_$+$iD>@~$N=E$_$K%j%;%C%H(B
                        weight = fast_naname_cost;
                    }
            		if ((map_naname[x][y] + weight) <= map_naname[x-1][y-1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x-1][y-1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x-1, y-1, south_west, south_west, weight);
                    }
            	}
            }

            if(x > 0)	// $B1&(B45$BEY2sE>(B
            {
            	if (wall_naname[x-1][y] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_45OUT_END_TIME; // $B8~$-$r(B45$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x-1][y] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x-1, y, south_west, west, weight);
                    }
            	}
            }

            if(y > 0)	// $B:8(B45$BEY2sE>(B
            {
            	if (wall_naname[x][y-1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_45OUT_END_TIME; // $B8~$-$r(B45$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x][y-1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x][y-1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x, y-1, south_west, south, weight);
                    }
            	}
            }

            if((x > 0) && (y < MAX_Y))	// $B1&(B90$BEY2sE>(B
            {
            	if (wall_naname[x-1][y+1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_END_TIME; // $B8~$-$r(B90$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y+1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x-1][y+1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x-1, y+1, south_west, north_west, weight);
                    }
            	}
            }

            if((x < MAX_X) && (y > 0))	// $B:8(B90$BEY2sE>(B
            {
            	if (wall_naname[x+1][y-1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_END_TIME; // $B8~$-$r(B90$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y-1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x+1][y-1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x+1, y-1, south_west, south_east, weight);
                    }
            	}
            }
            break;

        case east:
            if(x < MAX_X) 						//$BHO0O%A%'%C%/(B
            {
            	if (wall_naname[x+1][y] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    if(prev_dir != current_dir) { // $B8~$-$,JQ$o$C$?>l9g!"2sE>$N=E$_$+$iD>@~$N=E$_$K%j%;%C%H(B
                        weight = fast_straight_cost;
                    }
            		if ((map_naname[x][y] + weight) <= map_naname[x+1][y])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x+1][y] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x+1, y, east, east, weight);
                    }
            	}
            }

            if((y > 0) && (x < MAX_X))	// $B1&(B45$BEY2sE>(B
            {
            	if (wall_naname[x+1][y-1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_45IN_END_TIME; // $B8~$-$r(B45$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y-1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x+1][y-1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x+1, y-1, east, south_east, weight);
                    }
            	}
            }

            if((y < MAX_Y) && (x < MAX_X))	// $B:8(B45$BEY2sE>(B
            {
            	if (wall_naname[x+1][y+1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_45IN_END_TIME; // $B8~$-$r(B45$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y+1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x+1][y+1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x+1, y+1, east, north_east, weight);
                    }
            	}
            }

            if(y > 0)	// $B1&(B90$BEY2sE>(B
            {
            	if (wall_naname[x][y-1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_END_TIME; // $B8~$-$r(B90$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x][y-1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x][y-1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x, y-1, east, south, weight);
                    }
            	}
            }

            if(y < MAX_Y)	// $B:8(B90$BEY2sE>(B
            {
            	if (wall_naname[x][y+1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_END_TIME; // $B8~$-$r(B90$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x][y+1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x][y+1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x, y+1, east, north, weight);
                    }
            	}
            }
            break;

        case west:
            if(x > 0) 						//$BHO0O%A%'%C%/(B
            {
            	if (wall_naname[x-1][y] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    if(prev_dir != current_dir) { // $B8~$-$,JQ$o$C$?>l9g!"2sE>$N=E$_$+$iD>@~$N=E$_$K%j%;%C%H(B
                        weight = fast_straight_cost;
                    }
            		if ((map_naname[x][y] + weight) <= map_naname[x-1][y])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x-1][y] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x-1, y, west, west, weight);
                    }
            	}
            }

            if((y < MAX_Y) && (x > 0))	// $B1&(B45$BEY2sE>(B
            {
            	if (wall_naname[x-1][y+1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_45IN_END_TIME; // $B8~$-$r(B45$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y+1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x-1][y+1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x-1, y+1, west, north_west, weight);
                    }
            	}
            }

            if((y > 0) && (x > 0))	// $B:8(B45$BEY2sE>(B
            {
            	if (wall_naname[x-1][y-1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_45IN_END_TIME; // $B8~$-$r(B45$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y-1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x-1][y-1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x-1, y-1, west, south_west, weight);
                    }
            	}
            }

            if(y < MAX_Y)	// $B1&(B90$BEY2sE>(B
            {
            	if (wall_naname[x][y+1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_END_TIME; // $B8~$-$r(B90$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x][y+1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x][y+1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x, y+1, west, north, weight);
                    }
            	}
            }

            if(y > 0)	// $B:8(B90$BEY2sE>(B
            {
            	if (wall_naname[x][y-1] == NOWALL)//$BJI$,$J$1$l$P(B(mask$B$N0UL#$O(Bstatic_parameters$B$r;2>H(B)
            	{
                    weight = DAIKEI_END_TIME; // $B8~$-$r(B90$BEY2sE>$7$?$?$a!"(B45$BEY$N=E$_$K@_Dj(B
            		if((map_naname[x][y] + weight) <= map_naname[x][y-1])				//$B$^$@CM$,F~$C$F$$$J$1$l$P(B
            		{
            			map_naname[x][y-1] = map_naname[x][y] + weight;	//$BCM$rBeF~(B
                        prioritize_straight_cost_recursion(x, y-1, west, south, weight);
                    }
            	}
            }
            break;

    }
}



void make_map_naname_recursion(int x, int y)	//$BJb?t%^%C%W$r:n@.$9$k(B
{

	init_map_naname(x,y);											//Map$B$r=i4|2=$9$k(B
    
    prioritize_straight_cost_recursion(x*2+1, y*2+1, north, north, fast_straight_cost);
    prioritize_straight_cost_recursion(x*2+1, y*2+1, south, south, fast_straight_cost);
    prioritize_straight_cost_recursion(x*2+1, y*2+1, east, east, fast_straight_cost);
    prioritize_straight_cost_recursion(x*2+1, y*2+1, west, west, fast_straight_cost);
    prioritize_straight_cost_recursion(x*2+1, y*2+1, north_east, north_east, fast_naname_cost);
    prioritize_straight_cost_recursion(x*2+1, y*2+1, north_west, north_west, fast_naname_cost);
    prioritize_straight_cost_recursion(x*2+1, y*2+1, south_west, south_west, fast_naname_cost);
    prioritize_straight_cost_recursion(x*2+1, y*2+1, south_west, south_west, fast_naname_cost);
    /*
    char str[6];
    unsigned short score;
    for(int j = 0; j <MAZESIZE_Y; j++)						//$BLBO)$NBg$-$5J,%k!<%W(B(x$B:BI8(B)
    {
    	for(int i = 0; i < MAZESIZE_X; i++)					//$BLBO)$NBg$-$5J,%k!<%W(B(y$B:BI8(B)
    	{
                score = map_naname[i*2+1][j*2+1];

                sprintf(str, "%u", score);
                API_setText(i, j, str);
        }
    }
    for(int j = MAX_Y; j >= 0; j--)						//$BLBO)$NBg$-$5J,%k!<%W(B(x$B:BI8(B)
    {
    	for(int i = 0; i < MAX_X+1; i++)					//$BLBO)$NBg$-$5J,%k!<%W(B(y$B:BI8(B)
    	{
                score = map_naname[i][j];
                fprintf(stderr, "x:%d,y:%d,s:%u  ", i, j, score);
                fflush(stderr);
        }
                fprintf(stderr, "\n\n");
                fflush(stderr);
    }
    */
}



t_bool is_unknown(int x, int y)	//$B;XDj$5$l$?6h2h$,L$C5:w$+H]$+$rH=CG$9$k4X?t(B $BL$C5:w(B:true$B!!C5:w:Q(B:false
{
	//$B:BI8(Bx,y$B$,L$C5:w6h4V$+H]$+$rD4$Y$k(B
	
	if((wall[x][y].north == UNKNOWN) || (wall[x][y].east == UNKNOWN) || (wall[x][y].south == UNKNOWN) || (wall[x][y].west == UNKNOWN))//$B$I$3$+$NJI>pJs$,ITL@$N$^$^$G$"$l$P(B
	{			
		return true;	//$BL$C5:w(B
	}
	else
	{
		return false;	//$BC5:w:Q(B
	}
}



int get_priority(int x, int y, t_direction dir)	//$B$=$N%^%9$N>pJs$+$i!"M%@hEY$r;;=P$9$k(B
{
	//$B:BI8(Bx,y$B$H!"8~$$$F$$$kJ}3Q(Bdir$B$+$iM%@hEY$r;;=P$9$k(B
	
	//$BL$C5:w$,0lHVM%@hEY$,9b$$(B.(4)
	//$B$=$l$K2C$(!"<+J,$N8~$-$H!"9T$-$?$$J}8~$+$i!"(B
	//$BA0(B(2)$B2#(B(1)$B8e(B(0)$B$NM%@hEY$rIU2C$9$k!#(B

	int priority;								//$BM%@hEY$r5-O?$9$kJQ?t(B
	
	priority = 0;
	
	if(mypos.dir == dir)						//$B9T$-$?$$J}8~$,8=:_$N?J9TJ}8~$HF1$8>l9g(B
	{
		priority = 2;
	}
	else if( ((4+mypos.dir-dir)%4) == 2)		//$B9T$-$?$$J}8~$,8=:_$N?J9TJ}8~$H5U$N>l9g(B
	{
		priority = 0;
	}
	else										//$B$=$l0J30(B($B:81&$I$A$i$+(B)$B$N>l9g(B
	{
		priority = 1;
	}
	
	
	if(is_unknown(x,y) == true)
	{
		priority += 4;							//$BL$C5:w$N>l9gM%@hEY$r$5$i$KIU2C(B
	}
	
	return priority;							//$BM%@hEY$rJV$9(B
	
}



int get_nextdir_naname(int x, int y, t_direction *dir)	
{
	//$B%4!<%k:BI8(Bx,y$B$K8~$+$&>l9g!":#$I$A$i$K9T$/$Y$-$+$rH=CG$9$k!#(B
	int little,priority,tmp_priority;								//$B:G>.$NCM$rC5$9$?$a$K;HMQ$9$kJQ?t(B
 
	little = 59999;													//$B:G>.Jb?t$r(B255$BJb(B(map$B$,(Bunsigned char$B7?$J$N$G(B)$B$K@_Dj(B	

	priority = 0;													//$BM%@hEY$N=i4|CM$O(B0
	
	if( wall_naname[mypos.x*2+1][mypos.y*2+2] == NOWALL)			//$BKL$KJI$,$J$1$l$P(B
	{
		tmp_priority = get_priority(mypos.x, mypos.y + 1, north);	//$BM%@hEY$r;;=P(B
		if(map_naname[mypos.x*2+1][(mypos.y+1)*2+1] < little)						//$B0lHVJb?t$,>.$5$$J}8~$r8+$D$1$k(B
		{
			little = map_naname[mypos.x*2+1][(mypos.y+1)*2+1];						//$B$R$H$^$:KL$,Jb?t$,>.$5$$;v$K$9$k(B
			*dir = north;											//$BJ}8~$rJ]B8(B
			priority = tmp_priority;								//$BM%@hEY$rJ]B8(B
		}
		else if(map_naname[mypos.x*2+1][(mypos.y+1)*2+1] == little)					//$BJb?t$,F1$8>l9g$OM%@hEY$+$iH=CG$9$k(B
		{
			if(priority < tmp_priority )							//$BM%@hEY$rI>2A(B
			{
				*dir = north;										//$BJ}8~$r99?7(B
				priority = tmp_priority;							//$BM%@hEY$rJ]B8(B
			}
		}
	}
	
	if( wall_naname[mypos.x*2+2][mypos.y*2+1] == NOWALL)				//$BEl$KJI$,$J$1$l$P(B
	{
		tmp_priority = get_priority(mypos.x + 1, mypos.y, east);	//$BM%@hEY$r;;=P(B
		if(map_naname[(mypos.x + 1)*2+1][mypos.y*2+1] < little)						//$B0lHVJb?t$,>.$5$$J}8~$r8+$D$1$k(B
		{
			little = map_naname[(mypos.x+1)*2+1][mypos.y*2+1];						//$B$R$H$^$:El$,Jb?t$,>.$5$$;v$K$9$k(B
			*dir = east;											//$BJ}8~$rJ]B8(B
			priority = tmp_priority;								//$BM%@hEY$rJ]B8(B
		}
		else if(map_naname[(mypos.x + 1)*2+1][mypos.y*2+1] == little)				//$BJb?t$,F1$8>l9g!"M%@hEY$+$iH=CG(B
		{
			if(priority < tmp_priority)								//$BM%@hEY$rI>2A(B
			{
				*dir = east;										//$BJ}8~$rJ]B8(B
				priority = tmp_priority;							//$BM%@hEY$rJ]B8(B
			}
		}
	}
	
	if( wall_naname[mypos.x*2+1][mypos.y*2] == NOWALL)			//$BFn$KJI$,$J$1$l$P(B
	{
		tmp_priority = get_priority(mypos.x, mypos.y - 1, south);	//$BM%@hEY$r;;=P(B
		if(map_naname[mypos.x*2+1][(mypos.y - 1)*2+1] < little)						//$B0lHVJb?t$,>.$5$$J}8~$r8+$D$1$k(B
		{
			little = map_naname[mypos.x*2+1][(mypos.y-1)*2+1];						//$B$R$H$^$:Fn$,Jb?t$,>.$5$$;v$K$9$k(B
			*dir = south;											//$BJ}8~$rJ]B8(B
			priority = tmp_priority;								//$BM%@hEY$rJ]B8(B
		}
		else if(map_naname[mypos.x*2+1][(mypos.y - 1)*2+1] == little)				//$BJb?t$,F1$8>l9g!"M%@hEY$GI>2A(B
		{
			if(priority < tmp_priority)								//$BM%@hEY$rI>2A(B
			{
				*dir = south;										//$BJ}8~$rJ]B8(B
				priority = tmp_priority;							//$BM%@hEY$rJ]B8(B
			}
		}
	}
	
	if( wall_naname[mypos.x*2][mypos.y*2+1] == NOWALL)				//$B@>$KJI$,$J$1$l$P(B
	{
		tmp_priority = get_priority(mypos.x - 1, mypos.y, west);	//$BM%@hEY$r;;=P(B
		if(map_naname[(mypos.x-1)*2+1][mypos.y*2+1] < little)						//$B0lHVJb?t$,>.$5$$J}8~$r8+$D$1$k(B
		{
			little = map_naname[(mypos.x-1)*2+1][mypos.y*2+1];						//$B@>$,Jb?t$,>.$5$$(B
			*dir = west;											//$BJ}8~$rJ]B8(B
			priority = tmp_priority;								//$BM%@hEY$rJ]B8(B
		}
		else if(map_naname[(mypos.x - 1)*2+1][mypos.y*2+1] == little)				//$BJb?t$,F1$8>l9g!"M%@hEY$GI>2A(B
		{
			*dir = west;											//$BJ}8~$rJ]B8(B
			priority = tmp_priority;								//$BM%@hEY$rJ]B8(B
		}
	}

    if(little == 59999) {
	unable_to_find_path_to_goal = 1;
	return stop;
    }

	return ( (int)( ( 4 + *dir - mypos.dir) % 4 ) );				//$B$I$C$A$K8~$+$&$Y$-$+$rJV$9!#(B
}



int get_nextdir(int x, int y, int mask, t_direction *dir)	
{
	//$B%4!<%k:BI8(Bx,y$B$K8~$+$&>l9g!":#$I$A$i$K9T$/$Y$-$+$rH=CG$9$k!#(B
	//$BC5:w!":GC;$N@Z$jBX$($N$?$a$N(Bmask$B$r;XDj!"(Bdir$B$OJ}3Q$r<($9(B
	int little,priority,tmp_priority;								//$B:G>.$NCM$rC5$9$?$a$K;HMQ$9$kJQ?t(B
 
	make_map(x,y,mask);												//$BJb?t(BMap$B@8@.(B
	little = 255;													//$B:G>.Jb?t$r(B255$BJb(B(map$B$,(Bunsigned char$B7?$J$N$G(B)$B$K@_Dj(B	

	priority = 0;													//$BM%@hEY$N=i4|CM$O(B0
	
		//mask$B$N0UL#$O(Bstatic_parameter.h$B$r;2>H(B
	if( (wall[mypos.x][mypos.y].north & mask) == NOWALL)			//$BKL$KJI$,$J$1$l$P(B
	{
		tmp_priority = get_priority(mypos.x, mypos.y + 1, north);	//$BM%@hEY$r;;=P(B
		if(map[mypos.x][mypos.y+1] < little)						//$B0lHVJb?t$,>.$5$$J}8~$r8+$D$1$k(B
		{
			little = map[mypos.x][mypos.y+1];						//$B$R$H$^$:KL$,Jb?t$,>.$5$$;v$K$9$k(B
			*dir = north;											//$BJ}8~$rJ]B8(B
			priority = tmp_priority;								//$BM%@hEY$rJ]B8(B
		}
		else if(map[mypos.x][mypos.y+1] == little)					//$BJb?t$,F1$8>l9g$OM%@hEY$+$iH=CG$9$k(B
		{
			if(priority < tmp_priority )							//$BM%@hEY$rI>2A(B
			{
				*dir = north;										//$BJ}8~$r99?7(B
				priority = tmp_priority;							//$BM%@hEY$rJ]B8(B
			}
		}
	}
	
	if( (wall[mypos.x][mypos.y].east & mask) == NOWALL)				//$BEl$KJI$,$J$1$l$P(B
	{
		tmp_priority = get_priority(mypos.x + 1, mypos.y, east);	//$BM%@hEY$r;;=P(B
		if(map[mypos.x + 1][mypos.y] < little)						//$B0lHVJb?t$,>.$5$$J}8~$r8+$D$1$k(B
		{
			little = map[mypos.x+1][mypos.y];						//$B$R$H$^$:El$,Jb?t$,>.$5$$;v$K$9$k(B
			*dir = east;											//$BJ}8~$rJ]B8(B
			priority = tmp_priority;								//$BM%@hEY$rJ]B8(B
		}
		else if(map[mypos.x + 1][mypos.y] == little)				//$BJb?t$,F1$8>l9g!"M%@hEY$+$iH=CG(B
		{
			if(priority < tmp_priority)								//$BM%@hEY$rI>2A(B
			{
				*dir = east;										//$BJ}8~$rJ]B8(B
				priority = tmp_priority;							//$BM%@hEY$rJ]B8(B
			}
		}
	}
	
	if( (wall[mypos.x][mypos.y].south & mask) == NOWALL)			//$BFn$KJI$,$J$1$l$P(B
	{
		tmp_priority = get_priority(mypos.x, mypos.y - 1, south);	//$BM%@hEY$r;;=P(B
		if(map[mypos.x][mypos.y - 1] < little)						//$B0lHVJb?t$,>.$5$$J}8~$r8+$D$1$k(B
		{
			little = map[mypos.x][mypos.y-1];						//$B$R$H$^$:Fn$,Jb?t$,>.$5$$;v$K$9$k(B
			*dir = south;											//$BJ}8~$rJ]B8(B
			priority = tmp_priority;								//$BM%@hEY$rJ]B8(B
		}
		else if(map[mypos.x][mypos.y - 1] == little)				//$BJb?t$,F1$8>l9g!"M%@hEY$GI>2A(B
		{
			if(priority < tmp_priority)								//$BM%@hEY$rI>2A(B
			{
				*dir = south;										//$BJ}8~$rJ]B8(B
				priority = tmp_priority;							//$BM%@hEY$rJ]B8(B
			}
		}
	}
	
	if( (wall[mypos.x][mypos.y].west & mask) == NOWALL)				//$B@>$KJI$,$J$1$l$P(B
	{
		tmp_priority = get_priority(mypos.x - 1, mypos.y, west);	//$BM%@hEY$r;;=P(B
		if(map[mypos.x-1][mypos.y] < little)						//$B0lHVJb?t$,>.$5$$J}8~$r8+$D$1$k(B
		{
			little = map[mypos.x-1][mypos.y];						//$B@>$,Jb?t$,>.$5$$(B
			*dir = west;											//$BJ}8~$rJ]B8(B
			priority = tmp_priority;								//$BM%@hEY$rJ]B8(B
		}
		else if(map[mypos.x - 1][mypos.y] == little)				//$BJb?t$,F1$8>l9g!"M%@hEY$GI>2A(B
		{
			*dir = west;											//$BJ}8~$rJ]B8(B
			priority = tmp_priority;								//$BM%@hEY$rJ]B8(B
		}
	}

    if(little == 255) {
	unable_to_find_path_to_goal = 1;
	return stop;
    }

	return ( (int)( ( 4 + *dir - mypos.dir) % 4 ) );				//$B$I$C$A$K8~$+$&$Y$-$+$rJV$9!#(B
}



int fast_full_search_adachi_slalom(int startX, int endX, int startY, int endY)
{
//$B0z?t(Bgx,gy$B$K8~$+$C$FB-N)K!$GLBO)$rC5:w$9$k(B
    short tmp_x, tmp_y;
    short gx, gy;
    short no_goal_exit_loop_flag = 0;
	t_direction glob_nextdir;									//$B<!$K8~$+$&J}8~$r5-O?$9$kJQ?t(B

	static int search_straight_count = 0;
    already_passed_boolean_map[0] = 0; // set first position to zero inorder to get back to the start at the end of fullsearch
    int sStart = startX + startY; // Maximum s value for the current square
    int sEnd = endX + endY;       // Minimum s value for the current square

    // For traversal from bottom-right to top-left, s decreases
    for (int s = sStart; s >= sEnd; s--) {
        // Traverse diagonally for the current value of s
        for (int gx = startX; gx >= endX; gx--) {
            int gy = s - gx;
            // Make sure x and y are within the boundaries of the current square
            if ((gy <= startY) && (gy >= endY)) {

		    if ((already_passed_boolean_map[gy*16 + gx] == 1)){
	                continue; // pass to the next goal position
	            }//*/
		    /*if((is_unknown(gx,gy) == false) && ((gy!=0) && (gx!=0)))
		    {
			 continue; // pass to the next goal position
		    }//*/

	            switch(get_nextdir(gx,gy,MASK_SEARCH,&glob_nextdir))		//$B<!$K9T$/J}8~$rLa$jCM$H$9$k4X?t$r8F$V(B
	            {
	                case front:
                        fprintf(stderr, "front\n");
                        fflush(stderr);
	                    if(unable_to_find_path_to_goal == 1) {
                        fprintf(stderr, "Cant find path!!! \n");
                        fflush(stderr);
	                        unable_to_find_path_to_goal = 0;
	                        continue;
	                    }
			    
	                    //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);	//$B0l6h2h?J$`(B
                        API_moveForward();
	                    break;
	                
	                case right:
                        fprintf(stderr, "right\n");
                        fflush(stderr);
	                    //rotate(right,1);									//$B1&$K6J$,$C$F(B
	                    //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);	//$BH>6h2h?J$`(B
                        API_turnRight();
                        API_moveForward();
	                    break;
	                
	                case left:
                        fprintf(stderr, "left\n");
                        fflush(stderr);
	                    //rotate(left,1);										//$B:8$K6J$,$C$F(B
	                    //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);	//$BH>6h2h?J$`(B
                        API_turnLeft();
                        API_moveForward();
	                    break;
	                
	                case rear:
                        fprintf(stderr, "rear\n");
                        fflush(stderr);
	                    //rotate(left,2);										//180$B%?!<%s(B
	                    //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);	//$BH>6h2h?J$`(B
                        API_turnLeft();
                        API_turnLeft();
                        API_moveForward();
	                    break;
			case stop:
                fprintf(stderr, "STOP!!! \n");
                fflush(stderr);
			    if(unable_to_find_path_to_goal == 1) {
	                        unable_to_find_path_to_goal = 0;
			    }
	                    continue;
	                    
	            }
	            mypos.dir = glob_nextdir;									//$BJ}8~$r99?7(B


	            //$B8~$$$?J}8~$K$h$C$F<+J,$N:BI8$r99?7$9$k(B
	            switch(mypos.dir)
	            {
	                case north:
                        fprintf(stderr, "current direction: north \n");
                        fflush(stderr);
	                    mypos.y++;	//$BKL$r8~$$$?;~$O(BY$B:BI8$rA}$d$9(B
	                    break;
	                    
	                case east:
                        fprintf(stderr, "current direction: east\n");
	                    mypos.x++;	//$BEl$r8~$$$?;~$O(BX$B:BI8$rA}$d$9(B
	                    break;
	                    
	                case south:
                        fprintf(stderr, "current direction: south\n");
	                    mypos.y--;	//$BFn$r8~$$$?;~$O(BY$B:BI8$r8:$i$9(B
	                    break;
	                
	                case west:
                        fprintf(stderr, "current direction: west\n");
	                    mypos.x--;	//$B@>$r8~$$$?$H$-$O(BX$B:BI8$r8:$i$9(B
	                    break;

	            }
	            
	            while(((mypos.x != gx) || (mypos.y != gy)) && (!no_goal_exit_loop_flag)){							//$B%4!<%k$9$k$^$G7+$jJV$9(B
                    fprintf(stderr, "current x: %d, y: %d | goal:%d, %d \n", mypos.x, mypos.y, gx, gy);
                    fflush(stderr);
                    API_setColor(mypos.x, mypos.y, 'A');
		    	    if (already_passed_boolean_map[mypos.y*16 + mypos.x] == 0){
		                    already_passed_boolean_map[mypos.y*16 + mypos.x] = 1;
		                    set_wall(mypos.x,mypos.y);					//$BJI$r%;%C%H(B
	                }//*/
			   
			    if(is_unknown(mypos.x,mypos.y) == true)
		            {
				set_wall(mypos.x,mypos.y);                    //$BJI$r%;%C%H(B
			    }
			       

	                switch(get_nextdir(gx,gy,MASK_SEARCH,&glob_nextdir))			//$B<!$K9T$/J}8~$rLa$jCM$H$9$k4X?t$r8F$V(B
	                {
	                    case front:
                            fprintf(stderr, "front\n");
			    	        //switch(glob_nextdir) {
	                        //            case north:
	                        //                tmp_x = mypos.x;
	                        //                tmp_y = mypos.y + 1;
	                        //                break;
	                        //            case east:
	                        //                tmp_x = mypos.x + 1;
	                        //                tmp_y = mypos.y;
	                        //                break;
	                        //            case south:
	                        //                tmp_x = mypos.x;
	                        //                tmp_y = mypos.y - 1;
	                        //                break;
	                        //            case west:
	                        //                tmp_x = mypos.x - 1;
	                        //                tmp_y = mypos.y;
	                        //        }
	                        //if (already_passed_boolean_map[tmp_y*16 + tmp_x] == 1)	{//$B4{CN6h2h$r%9%-%C%W(B
	                        //    search_straight_count+=1;
	                        //    mypos.x=tmp_x;
	                        //    mypos.y=tmp_y;
	                        //    mypos.dir = glob_nextdir;
	                        //    continue;
	                        //} else {//$BL$CN6h2h(B
                        if (search_straight_count>0){
                            for(short z=0; z<(search_straight_count+1); z++){
                                API_moveForward();
                            }
                            API_moveForward();
					    //straight_for_search(SECTION*(search_straight_count+1),700.0,SEARCH_SPEED);
                        } else {
                            API_moveForward();
					    //straight_for_search(SECTION,SEARCH_SPEED,SEARCH_SPEED);	//$B0l6h2h?J$`(B
                        }
	                            search_straight_count=0;
	                        //}
	                            
	                        break;
	                    
	                    case right:
                            fprintf(stderr, "right\n");
	                        if (search_straight_count>1){
                                for(short z=0; z<(search_straight_count+1); z++){
                                API_moveForward();
                                }
	                            //straight_for_search(SECTION*search_straight_count,700.0,SEARCH_SPEED);
                            }
	                        //else{
                            //    API_moveForward();
	                        //    //straight_for_search(SECTION*search_straight_count,SEARCH_SPEED,SEARCH_SPEED);
                            //}
	                        //rotate_slalom(right, DAIKEI_ZENSHIN, DAIKEI_ZENSHIN);
                            API_turnRight();
                            API_moveForward();
	                        search_straight_count = 0;
	                        break;
	                    
	                    case left:
                            fprintf(stderr, "left\n");
	                        if (search_straight_count>1){
                                for(short z=0; z<(search_straight_count+1); z++){
                                API_moveForward();
                                }
	                            //straight_for_search(SECTION*search_straight_count,700.0,SEARCH_SPEED);
                            }
	                        //else{
                            //    API_moveForward();
                            //}
	                            //straight_for_search(SECTION*search_straight_count, SEARCH_SPEED,SEARCH_SPEED);
	                        //rotate_slalom(left, DAIKEI_ZENSHIN, DAIKEI_ZENSHIN);
                            API_turnLeft();
                            API_moveForward();
	                        search_straight_count = 0;
	                        break;
	                    
	                    case rear:
                            fprintf(stderr, "rear\n");
                            if (search_straight_count>0){
                                for(short z=0; z<(search_straight_count+1); z++){
                                API_moveForward();
                            }
					    //straight_for_search(SECTION*(search_straight_count+0.5),700.0,0);
                        } else {
                                API_moveForward();
                             //straight_for_search(HALF_SECTION,SEARCH_SPEED,0);
                        }
	                         
	                       		//$BH>6h2h?J$s$G(B
	                        //rotate(left,2);					//180$B%?!<%s(B
	                        //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);		//$BH>6h2h?J$`(B
                            switch(mypos.dir)
                            {
                                case north:
                                    API_setColor(mypos.x, mypos.y-1, 'A');
                                    already_passed_boolean_map[(mypos.y-1)*16 + mypos.x] =1;
                                    break;
                                    
                                case east:
                                    API_setColor(mypos.x-1, mypos.y, 'A');
                                    already_passed_boolean_map[mypos.y*16 + mypos.x-1] =1;
                                    break;
                                    
                                case south:
                                    API_setColor(mypos.x, mypos.y+1, 'A');
                                    already_passed_boolean_map[(mypos.y+1)*16 + mypos.x] =1;
                                    break;
                                
                                case west:
                                    API_setColor(mypos.x+1, mypos.y, 'A');
                                    already_passed_boolean_map[mypos.y*16 + mypos.x+1] =1;
                                    break;

                            }
                            
                            API_turnLeft();
                            API_turnLeft();
                            API_moveForward();
	                        search_straight_count = 0;
	                        break;
			case stop:
                fprintf(stderr, "STOP!!! \n");
                fflush(stderr);
			    if(unable_to_find_path_to_goal == 1) {
				    //search_straight_count=0;
				    unable_to_find_path_to_goal = 0;
				    }
			             no_goal_exit_loop_flag = 1;
				    continue;
	                    
	                }

	            
	                mypos.dir = glob_nextdir;										//$BJ}8~$r99?7(B
	                
	                //$B8~$$$?J}8~$K$h$C$F<+J,$N:BI8$r99?7$9$k(B
	                switch(mypos.dir)
	                {
	                    case north:
                            fprintf(stderr, "current dir: north\n");
                            fflush(stderr);
	                        mypos.y++;	//$BKL$r8~$$$?;~$O(BY$B:BI8$rA}$d$9(B
	                        break;
	                        
	                    case east:
                            fprintf(stderr, "current dir: east\n");
	                        mypos.x++;	//$BEl$r8~$$$?;~$O(BX$B:BI8$rA}$d$9(B
	                        break;
	                        
	                    case south:
                            fprintf(stderr, "current dir: south\n");
	                        mypos.y--;	//$BFn$r8~$$$?;~$O(BY$B:BI8$r8:$i$9(B
	                        break;
	                    
	                    case west:
                            fprintf(stderr, "current dir: west\n");
	                        mypos.x--;	//$B@>$r8~$$$?$H$-$O(BX$B:BI8$r8:$i$9(B
	                        break;

	                }
	                
	            }
		    if (((mypos.x == gx) && (mypos.y == gy)) && (already_passed_boolean_map[mypos.y*16 + mypos.x] == 0)){
                    fprintf(stderr, "current x: %d, y: %d | goal:%d, %d \n", mypos.x, mypos.y, gx, gy);
                    fprintf(stderr, "GOAL!!!!\n");
                    set_wall(mypos.x,mypos.y);                    //$BJI$r%;%C%H(B
                    API_clearAllText();
		            already_passed_boolean_map[mypos.y*16 + mypos.x] = 1;
	        }
		    no_goal_exit_loop_flag = 0;
	            if (search_straight_count>0){
                    for(short z=0; z<(search_straight_count); z++){
                        API_moveForward();
                    }
	                //straight_for_search(SECTION*search_straight_count+HALF_SECTION,700.0,SEARCH_SPEED);
	                search_straight_count=0;
				    
	            } else {
			if(is_unknown(mypos.x,mypos.y) == true)
		    	{
			 	set_wall(mypos.x,mypos.y);                    //$BJI$r%;%C%H(B
		   	}
	                //straight_for_search(HALF_SECTION,SEARCH_SPEED,0);	//$B%4!<%k(B.$BH>6h2h?J$`(B
	            }
		  
	    }
        }
    }
    //*/
    return 0;
}



//$B%9%i%m!<%`Av9T$N:GC;7PO)%^%C%W$r:n@.(B
void create_fast_run_slalom_map(int x, int y)
{
	t_direction glob_nextdir;
	float straight_count = -0.5;
    int action_id = 0;
    shortest_route_action[action_id] = M_FRONT;
    shortest_route_action_times[action_id] = straight_count;

	//$B8=:_$N8~$-$+$i!"<!$K9T$/$Y$-J}8~$X8~$/(B
	switch(get_nextdir(x,y,MASK_SECOND,&glob_nextdir))	//$B<!$K9T$/J}8~$rLa$jCM$H$9$k4X?t$r8F$V(B
	{
		case front:
			straight_count++;							//$BA08~$-$@$C$?>l9g$OD>@~$rAv$k5wN%$r?-$P$9(B
			break;
		
		case right:										//$B1&$K8~$/(B
			straight_count++;							//$BA08~$-$@$C$?>l9g$OD>@~$rAv$k5wN%$r?-$P$9(B
			break;
		
		case left:										//$B:8$K8~$/(B
			straight_count++;							//$BA08~$-$@$C$?>l9g$OD>@~$rAv$k5wN%$r?-$P$9(B
			break;
		
		case rear:										//$B8e$m$K8~$/(B
			straight_count++;							//$BA08~$-$@$C$?>l9g$OD>@~$rAv$k5wN%$r?-$P$9(B
			break;
	}

    shortest_route_action[action_id] = M_FRONT;
    shortest_route_action_times[action_id] = straight_count;
	mypos.dir = glob_nextdir;							//$B<+J,$N8~$-$r99?7(B


	//$B8~$$$?J}8~$K$h$C$F<+J,$N:BI8$r99?7$9$k(B
	switch(mypos.dir)
	{
		case north:
			mypos.y++;	//$BKL$r8~$$$?;~$O(BY$B:BI8$rA}$d$9(B
			break;
			
		case east:
			mypos.x++;	//$BEl$r8~$$$?;~$O(BX$B:BI8$rA}$d$9(B
			break;
			
		case south:
			mypos.y--;	//$BFn$r8~$$$?;~$O(BY$B:BI8$r8:$i$9(B
			break;
		
		case west:
			mypos.x--;	//$B@>$r8~$$$?$H$-$O(BX$B:BI8$r8:$i$9(B
			break;
	}

	while((mypos.x != x) || (mypos.y != y)){	//$B%4!<%k$9$k$^$G7+$jJV$9(B
		switch(get_nextdir(x,y,MASK_SECOND,&glob_nextdir))	//$B<!$K9T$/J}8~$rLa$jCM$H$9$k4X?t$r8F$V(B
		{
			case front:	//$BD>@~$r$^$H$a$FAv$k$h$&$K$9$k(B
				straight_count += 1.0;
				break;
			
			case right:
		                if (straight_count > 0.0) {
		                    //$BA0?J$rEPO?(B
		                    shortest_route_action[action_id] = M_FRONT;
		                    shortest_route_action_times[action_id] = straight_count;
		                    action_id++;

		                    //$B2sE>$rEPO?(B
		                    shortest_route_action[action_id] = M_RIGHT;
		                    shortest_route_action_times[action_id] = 1;
		                    action_id++;
		                    straight_count = 0.0;
		                }
		                else {
		                    //$B2sE>$rEPO?(B
		                    shortest_route_action[action_id] = M_RIGHT;
		                    shortest_route_action_times[action_id] = 1;
		                    action_id++;
		                }
				break;
			
			case left:
		                if (straight_count > 0.0) {
		                    //$BA0?J$rEPO?(B
		                    shortest_route_action[action_id] = M_FRONT;
		                    shortest_route_action_times[action_id] = straight_count;
		                    action_id++;

		                    //$B2sE>$rEPO?(B
		                    shortest_route_action[action_id] = M_LEFT;
		                    shortest_route_action_times[action_id] = 1;
		                    action_id++;
		                    straight_count = 0.0;
		                }
		                else {
		                    //$B2sE>$rEPO?(B
		                    shortest_route_action[action_id] = M_LEFT;
		                    shortest_route_action_times[action_id] = 1;
		                    action_id++;
		                }
		}
	
		mypos.dir = glob_nextdir;							//$B<+J,$N8~$-$r=$@5(B
		
		//$B8~$$$?J}8~$K$h$C$F<+J,$N:BI8$r99?7$9$k(B
		switch(mypos.dir)
		{
			case north:
				mypos.y++;	//$BKL$r8~$$$?;~$O(BY$B:BI8$rA}$d$9(B
				break;
				
			case east:
				mypos.x++;	//$BEl$r8~$$$?;~$O(BX$B:BI8$rA}$d$9(B
				break;
				
			case south:
				mypos.y--;	//$BFn$r8~$$$?;~$O(BY$B:BI8$r8:$i$9(B
				break;
			
			case west:
				mypos.x--;	//$B@>$r8~$$$?$H$-$O(BX$B:BI8$r8:$i$9(B
				break;

		}
	}
    shortest_route_action[action_id] = M_FRONT;
    shortest_route_action_times[action_id] = straight_count + 0.5;
    action_id++;
    shortest_route_action[action_id] = M_REAR;
}


//$B:GB.<P$a7PO)MQ$N%9%i%m!<%`Av9T$N:GC;7PO)%^%C%W$r:n@.(B
void create_fast_run_diagonal_map(int x, int y)
{
	t_direction glob_nextdir;
	float straight_count = -0.5;
    int action_id = 0;
    shortest_route_action[action_id] = M_FRONT;
    shortest_route_action_times[action_id] = straight_count;

    API_setColor(mypos.x, mypos.y, color);
	//$B8=:_$N8~$-$+$i!"<!$K9T$/$Y$-J}8~$X8~$/(B
	switch(get_nextdir_naname(x,y,&glob_nextdir))	//$B<!$K9T$/J}8~$rLa$jCM$H$9$k4X?t$r8F$V(B
	{
		case front:
			straight_count++;							//$BA08~$-$@$C$?>l9g$OD>@~$rAv$k5wN%$r?-$P$9(B
			break;
		
		case right:										//$B1&$K8~$/(B
			straight_count++;							//$BA08~$-$@$C$?>l9g$OD>@~$rAv$k5wN%$r?-$P$9(B
			break;
		
		case left:										//$B:8$K8~$/(B
			straight_count++;							//$BA08~$-$@$C$?>l9g$OD>@~$rAv$k5wN%$r?-$P$9(B
			break;
		
		case rear:										//$B8e$m$K8~$/(B
			straight_count++;							//$BA08~$-$@$C$?>l9g$OD>@~$rAv$k5wN%$r?-$P$9(B
			break;
	}

    shortest_route_action[action_id] = M_FRONT;
    shortest_route_action_times[action_id] = straight_count;
	mypos.dir = glob_nextdir;							//$B<+J,$N8~$-$r99?7(B


	//$B8~$$$?J}8~$K$h$C$F<+J,$N:BI8$r99?7$9$k(B
	switch(mypos.dir)
	{
		case north:
			mypos.y++;	//$BKL$r8~$$$?;~$O(BY$B:BI8$rA}$d$9(B
			break;
			
		case east:
			mypos.x++;	//$BEl$r8~$$$?;~$O(BX$B:BI8$rA}$d$9(B
			break;
			
		case south:
			mypos.y--;	//$BFn$r8~$$$?;~$O(BY$B:BI8$r8:$i$9(B
			break;
		
		case west:
			mypos.x--;	//$B@>$r8~$$$?$H$-$O(BX$B:BI8$r8:$i$9(B
			break;
	}

	while((mypos.x != x) || (mypos.y != y)){	//$B%4!<%k$9$k$^$G7+$jJV$9(B
        API_setColor(mypos.x, mypos.y, color);
		switch(get_nextdir_naname(x,y,&glob_nextdir))	//$B<!$K9T$/J}8~$rLa$jCM$H$9$k4X?t$r8F$V(B
		{
			case front:	//$BD>@~$r$^$H$a$FAv$k$h$&$K$9$k(B
				straight_count += 1.0;
				break;
			
			case right:
		                if (straight_count > 0.0) {
		                    //$BA0?J$rEPO?(B
		                    shortest_route_action[action_id] = M_FRONT;
		                    shortest_route_action_times[action_id] = straight_count;
		                    action_id++;

		                    //$B2sE>$rEPO?(B
		                    shortest_route_action[action_id] = M_RIGHT;
		                    shortest_route_action_times[action_id] = 1;
		                    action_id++;
		                    straight_count = 0.0;
		                }
		                else {
		                    //$B2sE>$rEPO?(B
		                    shortest_route_action[action_id] = M_RIGHT;
		                    shortest_route_action_times[action_id] = 1;
		                    action_id++;
		                }
				break;
			
			case left:
		                if (straight_count > 0.0) {
		                    //$BA0?J$rEPO?(B
		                    shortest_route_action[action_id] = M_FRONT;
		                    shortest_route_action_times[action_id] = straight_count;
		                    action_id++;

		                    //$B2sE>$rEPO?(B
		                    shortest_route_action[action_id] = M_LEFT;
		                    shortest_route_action_times[action_id] = 1;
		                    action_id++;
		                    straight_count = 0.0;
		                }
		                else {
		                    //$B2sE>$rEPO?(B
		                    shortest_route_action[action_id] = M_LEFT;
		                    shortest_route_action_times[action_id] = 1;
		                    action_id++;
		                }
		}
	
		mypos.dir = glob_nextdir;							//$B<+J,$N8~$-$r=$@5(B
		
		//$B8~$$$?J}8~$K$h$C$F<+J,$N:BI8$r99?7$9$k(B
		switch(mypos.dir)
		{
			case north:
				mypos.y++;	//$BKL$r8~$$$?;~$O(BY$B:BI8$rA}$d$9(B
				break;
				
			case east:
				mypos.x++;	//$BEl$r8~$$$?;~$O(BX$B:BI8$rA}$d$9(B
				break;
				
			case south:
				mypos.y--;	//$BFn$r8~$$$?;~$O(BY$B:BI8$r8:$i$9(B
				break;
			
			case west:
				mypos.x--;	//$B@>$r8~$$$?$H$-$O(BX$B:BI8$r8:$i$9(B
				break;

		}
	}
    API_setColor(mypos.x, mypos.y, color);
    shortest_route_action[action_id] = M_FRONT;
    shortest_route_action_times[action_id] = straight_count + 0.5;
    action_id++;
    shortest_route_action[action_id] = M_REAR;
}


void convert_shortest_route_to_diagonal_route(void)
{
    int action_id = 0;
    int diagonal_action_id = 0;
    int current_dir = M_FRONT;
    
    //if (shortest_route_action_times[action_id] <= 1.0) {
	//diagonal_route_action[diagonal_action_id] = M_FRONT;
    //    diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id];
    //    diagonal_action_id++;
    //    action_id++;
	//
	//while(shortest_route_action[action_id] != M_FRONT) {
    //            diagonal_route_action[diagonal_action_id] = shortest_route_action[action_id];
    //            diagonal_route_action_times[diagonal_action_id] = 1.0;
    //            diagonal_action_id++;
	//	action_id++;
	//}
	//shortest_route_action_times[action_id] -= 0.5;
    //}
    
    while(shortest_route_action[action_id] != M_REAR) {
        //fprintf(stderr, "action id of convert to diagonal_route %d\n", action_id);
        //fflush(stderr);
        switch(shortest_route_action[action_id])	//$B<!$K9T$/J}8~$rLa$jCM$H$9$k4X?t$r8F$V(B
    	{
	    case M_FRONT:
            fflush(stderr);
            //$B8=:_$N;Q@*$,1&<P$a>uBV(B
            if (current_dir == M_RIGHT45) { //45$BEY=P8}(B
                current_dir = M_FRONT; //$B;Q@*$r99?7(B
                diagonal_route_action[diagonal_action_id] = M_LEFT45OUT;
                diagonal_route_action_times[diagonal_action_id] = 1.0;
                diagonal_action_id++;

                // $B<P$a(B45$BEY=P8}8e!";Q@*$r=$@5$9$k$?$a$N6h2h$r3NJ](B
                shortest_route_action_times[action_id] -= 0.5;
            }

            //$B8=:_$N;Q@*$,:8<P$a>uBV(B
            if (current_dir == M_LEFT45) { //45$BEY=P8}(B
                current_dir = M_FRONT; //$B;Q@*$r99?7(B
                diagonal_route_action[diagonal_action_id] = M_RIGHT45OUT;
                diagonal_route_action_times[diagonal_action_id] = 1.0;
                diagonal_action_id++;

                // $B<P$a(B45$BEY=P8}8e!";Q@*$r=$@5$9$k$?$a$N6h2h$r3NJ](B
                shortest_route_action_times[action_id] -= 0.5;
            }

            //$B0J2<$N8=:_;Q@*$O??$CD>$0$N>uBV(B
            //$B:G8e$ND>@~$G$"$k$+$I$&$+$N3NG'(B
            if (shortest_route_action[action_id+1] == M_REAR) {
                diagonal_route_action[diagonal_action_id] = M_FRONT;
                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id];
                diagonal_action_id++;
                action_id++;

                diagonal_route_action[diagonal_action_id] = M_REAR;
                break;
            }

            //$B:8(B45$BEY<P$a3+;O(B
            //$B?J"*:8"*1&(B 
            if ((shortest_route_action[action_id+1] == M_LEFT) && (shortest_route_action[action_id+2] == M_RIGHT)) {
                diagonal_route_action[diagonal_action_id] = M_FRONT;
                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id] - 0.5; //45$BEY<P$a2sE>$N=`Hw6h2h(B
                diagonal_action_id++;

                current_dir = M_LEFT45;//$B;Q@*$r99?7(B
                diagonal_route_action[diagonal_action_id] = M_LEFT45;
                diagonal_route_action_times[diagonal_action_id] = 1.0;
                diagonal_action_id++;

                diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                diagonal_route_action_times[diagonal_action_id] = 1.0;
                diagonal_action_id++;

                action_id += 3; //$B<!$N9TF0$^$G%9%-%C%W(B
                break;
            } 
            
            //$B1&(B45$BEY<P$a3+;O(B
            //$B?J"*1&"*:8(B 
            if ((shortest_route_action[action_id+1] == M_RIGHT) && (shortest_route_action[action_id+2] == M_LEFT)) {
                diagonal_route_action[diagonal_action_id] = M_FRONT;
                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id] - 0.5; //45$BEY<P$a2sE>$N=`Hw6h2h(B
                diagonal_action_id++;

                current_dir = M_RIGHT45;//$B;Q@*$r99?7(B
                diagonal_route_action[diagonal_action_id] = M_RIGHT45;
                diagonal_route_action_times[diagonal_action_id] = 1.0;
                diagonal_action_id++;

                diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                diagonal_route_action_times[diagonal_action_id] = 1.0;
                diagonal_action_id++;

                action_id += 3; //$B<!$N9TF0$^$G%9%-%C%W(B
                break;
            }
            
            //$B:8(B90$BEY2sE>(B
            //$B?J"*:8"*?J(B
            if ((shortest_route_action[action_id+1] == M_LEFT) && (shortest_route_action[action_id+2] == M_FRONT)) {	  
		//$B:8(B90$BEY2sE>Bg2s$j(B
                /*if (shortest_route_action_times[action_id] >= 2.0 && shortest_route_action_times[action_id+2] >= 2.5) {
	    		diagonal_route_action[diagonal_action_id] = M_FRONT;
	                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id] - 0.5;
	                diagonal_action_id++;

	                current_dir = M_FRONT;//$B;Q@*$r99?7(B
	                diagonal_route_action[diagonal_action_id] = M_LEFT_F;
	                diagonal_route_action_times[diagonal_action_id] = 1.0;
	                diagonal_action_id++;

	                shortest_route_action_times[action_id+2] -= 1.5;
                } else {*/
	                diagonal_route_action[diagonal_action_id] = M_FRONT;
	                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id];
	                diagonal_action_id++;

	                current_dir = M_FRONT;//$B;Q@*$r99?7(B
	                diagonal_route_action[diagonal_action_id] = M_LEFT;
	                diagonal_route_action_times[diagonal_action_id] = 1.0;
	                diagonal_action_id++;

	                shortest_route_action_times[action_id+2] -= 0.25;
		//}
                action_id += 2; //$B<!$ND>@~9TF0$^$G%9%-%C%W(B
                break;
            }

            //$B1&(B90$BEY2sE>(B
            //$B?J"*1&"*?J(B
            if ((shortest_route_action[action_id+1] == M_RIGHT) && (shortest_route_action[action_id+2] == M_FRONT)) {
                /*if (shortest_route_action_times[action_id] >= 2.0 && shortest_route_action_times[action_id+2] >= 2.5) {
	    		diagonal_route_action[diagonal_action_id] = M_FRONT;
	                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id] - 0.5;
	                diagonal_action_id++;

	                current_dir = M_FRONT;//$B;Q@*$r99?7(B
	                diagonal_route_action[diagonal_action_id] = M_RIGHT_F;
	                diagonal_route_action_times[diagonal_action_id] = 1.0;
	                diagonal_action_id++;

	                shortest_route_action_times[action_id+2] -= 1.5;
                } else {*/
	                diagonal_route_action[diagonal_action_id] = M_FRONT;
	                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id];
	                diagonal_action_id++;

	                current_dir = M_FRONT; //$B;Q@*$r99?7(B
	                diagonal_route_action[diagonal_action_id] = M_RIGHT;
	                diagonal_route_action_times[diagonal_action_id] = 1.0;
	                diagonal_action_id++;

	                shortest_route_action_times[action_id+2] -= 0.25;
		//}
                action_id += 2; //$B<!$N9TF0$^$G%9%-%C%W(B
                break;
            }

            //$B:8(B135$BEY2sE>(B
            //$B?J"*:8"*:8"*1&(B
            if ((shortest_route_action[action_id+1] == M_LEFT) && (shortest_route_action[action_id+2] == M_LEFT) && (shortest_route_action[action_id+3] == M_RIGHT)) {
                diagonal_route_action[diagonal_action_id] = M_FRONT;
                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id]-0.5;
                diagonal_action_id++;

                current_dir = M_LEFT45; //$B;Q@*$r99?7!"(B90+$B<P$a(B45$BEY$G$"$k$?$a!";Q@*$O<P$a(B45$BEY$HF1$8(B
                diagonal_route_action[diagonal_action_id] = M_LEFT135;
                diagonal_route_action_times[diagonal_action_id] = 1.0;
                diagonal_action_id++;

                // $B:8(B135$B"*1&(BV90$B$G$J$$;~(B
                if (shortest_route_action[action_id+4] != M_RIGHT) {
                    diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                    diagonal_route_action_times[diagonal_action_id] = 1.0;
                    diagonal_action_id++;
                }

                action_id += 4; //$B<!$N9TF0$^$G%9%-%C%W(B
                break;
            }

            //$B1&(B135$BEY2sE>(B
            //$B?J"*1&"*1&"*:8(B
            if ((shortest_route_action[action_id+1] == M_RIGHT) && (shortest_route_action[action_id+2] == M_RIGHT) && (shortest_route_action[action_id+3] == M_LEFT)) {
                diagonal_route_action[diagonal_action_id] = M_FRONT;
                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id]-0.5;
                diagonal_action_id++;

                current_dir = M_RIGHT45; //$B;Q@*$r99?7!"(B90+$B<P$a(B45$BEY$G$"$k$?$a!";Q@*$O<P$a(B45$BEY$HF1$8(B
                diagonal_route_action[diagonal_action_id] = M_RIGHT135;
                diagonal_route_action_times[diagonal_action_id] = 1.0;
                diagonal_action_id++;

                // $B1&(B135$B"*:8(BV90$B$G$J$$;~(B
                if (shortest_route_action[action_id+4] != M_LEFT) {
                    diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                    diagonal_route_action_times[diagonal_action_id] = 1.0;
                    diagonal_action_id++;
                }

                action_id += 4; //$B<!$N9TF0$^$G%9%-%C%W(B
                break;
            }

            //$B:8(B180$BEY2sE>(B
            //$B?J"*:8"*:8"*?J(B
            if ((shortest_route_action[action_id+1] == M_LEFT) && (shortest_route_action[action_id+2] == M_LEFT) && (shortest_route_action[action_id+3] == M_FRONT)) {
                /*if ((shortest_route_action_times[action_id+3] - 0.5) >= 1.0) {
			diagonal_route_action[diagonal_action_id] = M_FRONT;
	                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id]-0.2;
	                diagonal_action_id++;

	                current_dir = M_FRONT; //$B;Q@*$r99?7(B
	                diagonal_route_action[diagonal_action_id] = M_LEFT180_F;
	                diagonal_route_action_times[diagonal_action_id] = 1.0;
	                diagonal_action_id++;
	                shortest_route_action_times[action_id+3] -= 0.01;
	                //shortest_route_action_times[action_id+3] -= 0.7;
		} else {*/
		        diagonal_route_action[diagonal_action_id] = M_FRONT;
	                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id]-0.1;
	                diagonal_action_id++;

	                current_dir = M_FRONT; //$B;Q@*$r99?7(B
	                diagonal_route_action[diagonal_action_id] = M_LEFT180;
	                diagonal_route_action_times[diagonal_action_id] = 1.0;
	                diagonal_action_id++;
			shortest_route_action_times[action_id+3] -= (0.25 + 0.07);
	                //shortest_route_action_times[action_id+3] -= 0.5;
		//}
                action_id += 3; //$B<!$N9TF0$^$G%9%-%C%W(B
                break;
            }

            //$B1&(B180$BEY2sE>(B
            //$B?J"*1&"*1&"*?J(B
            if ((shortest_route_action[action_id+1] == M_RIGHT) && (shortest_route_action[action_id+2] == M_RIGHT) && (shortest_route_action[action_id+3] == M_FRONT)) {
		/*if ((shortest_route_action_times[action_id+3] - 0.5) >= 1.0) {
			diagonal_route_action[diagonal_action_id] = M_FRONT;
	                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id]-0.2;
	                diagonal_action_id++;

	                current_dir = M_FRONT; //$B;Q@*$r99?7(B
	                diagonal_route_action[diagonal_action_id] = M_RIGHT180_F;
	                diagonal_route_action_times[diagonal_action_id] = 1.0;
	                diagonal_action_id++;
	                shortest_route_action_times[action_id+3] -= 0.01;
	                //shortest_route_action_times[action_id+3] -= 0.7;
		} else {*/
	                diagonal_route_action[diagonal_action_id] = M_FRONT;
	                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id]-0.1;
	                diagonal_action_id++;

	                current_dir = M_FRONT; //$B;Q@*$r99?7(B
	                diagonal_route_action[diagonal_action_id] = M_RIGHT180;
	                diagonal_route_action_times[diagonal_action_id] = 1.0;
	                diagonal_action_id++;
			shortest_route_action_times[action_id+3] -= (0.25 + 0.07);
	                //shortest_route_action_times[action_id+3] -= 0.5;
		//}
                action_id += 3; //$B<!$N9TF0$^$G%9%-%C%W(B
                break;
            }

	        diagonal_route_action[diagonal_action_id] = M_FRONT;
	        diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id];
	        diagonal_action_id++;
	        action_id++;
	        current_dir = M_FRONT;//$B;Q@*$r99?7(B
		
        //##################################$B1&2sE>(B######################################
		case M_RIGHT:
            if (current_dir == M_FRONT) {break;}

            if (current_dir == M_LEFT45) { //$B1&2sE>D>8e$K1&2sE>$9$k>uBV(B(left45==$B1&2sE>D>8e(B) $B"*(B $B1&(B+$B1&>uBV(B
                //$B1&"*1&"*?J(B == $B1&(BV90$BEY(B1$B2sE>(B $B"*(B $B1&(B45$BEY(B(45$BEY$O(Bfront$B$N%1!<%9$G9T$&(B)
                if (shortest_route_action[action_id+1] == M_FRONT) {
                    diagonal_route_action[diagonal_action_id] = M_RIGHTV90;
                    diagonal_route_action_times[diagonal_action_id] = 1.0;
                    diagonal_action_id++;

                    action_id += 1; //$B<!$N9TF0$^$G%9%-%C%W(B
                    break;
                }

                //$B1&"*1&"*:8(B == $B1&(BV90$BEY2sE>(B+ $B<P$aA0?J(B
                if (shortest_route_action[action_id+1] == M_LEFT) {
                    current_dir = M_RIGHT45; //45$BEY<P$a$HF1$8;Q@*$K=$@5(B
                    diagonal_route_action[diagonal_action_id] = M_RIGHTV90;
                    diagonal_route_action_times[diagonal_action_id] = 1.0;
                    diagonal_action_id++;

                    diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                    diagonal_route_action_times[diagonal_action_id] = 1.0;
                    diagonal_action_id++;

                    action_id += 2; //$B<!$N9TF0$^$G%9%-%C%W(B
                    break;
                }
            }

            if (current_dir == M_RIGHT45) {//$B:82sE>D>8e$K1&2sE>$9$k>uBV(B(left45==$B1&2sE>D>8e(B) $B"*(B $B:8(B+$B1&>uBV(B
                //$B:8"*1&"*?J(B == $B<P$aA0?J(B $B"*(B $B1&(B45$BEY(B(45$BEY$O(Bfront$B$N%1!<%9$G9T$&(B)
                if ((shortest_route_action[action_id+1] == M_FRONT)) {
                    current_dir = M_LEFT45; //$B<!$N%9%F%C%W$G;Q@*$r??$CD>$0$KD>$9$?$a$K!"(B45$BEY<P$a$HF1$8;Q@*$K=$@5(B
                    if (diagonal_route_action[diagonal_action_id-1] == M_FRONT_DIAGONAL) {
                        diagonal_route_action_times[diagonal_action_id-1] += 1.0;
                    } else {
                        diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                        diagonal_route_action_times[diagonal_action_id] = 1.0;
                        diagonal_action_id++;
                    }

                    action_id += 1; //$B<!$N9TF0$^$G%9%-%C%W(B
                    break;
                }

                //$B:8"*1&"*:8(B == $B<P$aA0?J(Bx2 
                if (shortest_route_action[action_id+1] == M_LEFT) {
                    if (diagonal_route_action[diagonal_action_id-1] == M_FRONT_DIAGONAL) {
                        diagonal_route_action_times[diagonal_action_id-1] += 2.0;
                    } else {
                        diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                        diagonal_route_action_times[diagonal_action_id] = 2.0;
                        diagonal_action_id++;
                    }

                    action_id += 2; //$B<!$N9TF0$^$G%9%-%C%W(B
                    break;
                }

                //$B:8"*1&"*1&(B == $B<P$aA0?J(B + $B1&(BV90
                if (shortest_route_action[action_id+1] == M_RIGHT) {
                    current_dir = M_LEFT45; //$B<!$N%9%F%C%W$G;Q@*$r??$CD>$0$KD>$9$?$a$K!"(B45$BEY<P$a$HF1$8;Q@*$K=$@5(B
                    if (diagonal_route_action[diagonal_action_id-1] == M_FRONT_DIAGONAL) {
                        diagonal_route_action_times[diagonal_action_id-1] += 1.0;

                        diagonal_route_action[diagonal_action_id] = M_RIGHTV90;
                        diagonal_route_action_times[diagonal_action_id] = 1.0;
                        diagonal_action_id++;
                    } else {
                        diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                        diagonal_route_action_times[diagonal_action_id] = 1.0;
                        diagonal_action_id++;

                        diagonal_route_action[diagonal_action_id] = M_RIGHTV90;
                        diagonal_route_action_times[diagonal_action_id] = 1.0;
                        diagonal_action_id++;
                    }

                    action_id += 2; //$B<!$N9TF0$^$G%9%-%C%W(B
                    break;
                }
            }
		
        //##################################$B:82sE>(B######################################
		case M_LEFT:
            //$B;Q@*<P$a>uBV$G:82sE>(B
            if (current_dir == M_FRONT) {break;}

            if (current_dir == M_LEFT45) { //$B1&2sE>D>8e$K:82sE>$9$k>uBV(B(left45==$B1&2sE>D>8e(B) $B"*(B $B1&(B+$B:8>uBV(B
                //$B1&"*:8"*?J(B == $B<P$aA0?J(B + $B:8(B45$BEY(B
                if (shortest_route_action[action_id+1] == M_FRONT) {
                    current_dir = M_RIGHT45; //45$BEY<P$a$HF1$8;Q@*$K=$@5(B
                    if (diagonal_route_action[diagonal_action_id-1] == M_FRONT_DIAGONAL) {
                        diagonal_route_action_times[diagonal_action_id-1] += 1.0;
                    } else {
                        diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                        diagonal_route_action_times[diagonal_action_id] = 1.0;
                        diagonal_action_id++;
                    }

                    action_id += 1; //$B<!$N9TF0$^$G%9%-%C%W(B
                    break;
                }

                //$B1&"*:8"*:8(B == $B<P$aA0?J(B + $B:8(BV90$BEY(B
                if (shortest_route_action[action_id+1] == M_LEFT) {
                    current_dir = M_RIGHT45; //$B<!$N%9%F%C%W$G;Q@*$r??$CD>$0$KD>$9$?$a$K!"(B45$BEY<P$a$HF1$8;Q@*$K=$@5(B
                    if (diagonal_route_action[diagonal_action_id-1] == M_FRONT_DIAGONAL) {
                        diagonal_route_action_times[diagonal_action_id-1] += 1.0;

                        diagonal_route_action[diagonal_action_id] = M_LEFTV90;
                        diagonal_route_action_times[diagonal_action_id] = 1.0;
                        diagonal_action_id++;
                    } else {
                        diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                        diagonal_route_action_times[diagonal_action_id] = 1.0;
                        diagonal_action_id++;

                        diagonal_route_action[diagonal_action_id] = M_LEFTV90;
                        diagonal_route_action_times[diagonal_action_id] = 1.0;
                        diagonal_action_id++;
                    }

                    action_id += 2; //$B<!$N9TF0$^$G%9%-%C%W(B
                    break;
                }

                //$B1&"*:8"*1&(B == $B<P$aA0?J(Bx2
                if (shortest_route_action[action_id+1] == M_RIGHT) {
                    if (diagonal_route_action[diagonal_action_id-1] == M_FRONT_DIAGONAL) {
                        diagonal_route_action_times[diagonal_action_id-1] += 2.0;
                    } else {
                        diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                        diagonal_route_action_times[diagonal_action_id] = 2.0;
                        diagonal_action_id++;
                    }

                    action_id += 2; //$B<!$N9TF0$^$G%9%-%C%W(B
                    break;
                }
            }

            if (current_dir == M_RIGHT45) {//$B:82sE>D>8e$K:82sE>$9$k>uBV(B(left45==$B1&2sE>D>8e(B) $B"*(B $B:8(B+$B:8>uBV(B
                //$B:8"*:8"*?J(B == $B:8(BV90 + $B1&(B45$BEY(B
                if (shortest_route_action[action_id+1] == M_FRONT) {
                    diagonal_route_action[diagonal_action_id] = M_LEFTV90;
                    diagonal_route_action_times[diagonal_action_id] = 1.0;
                    diagonal_action_id++;

                    action_id += 1; //$B<!$N9TF0$^$G%9%-%C%W(B
                    break;
                }

                //$B:8"*:8"*1&(B == $B:8(BV90 + $B<P$aA0?J(B
                if (shortest_route_action[action_id+1] == M_RIGHT) {
                    current_dir = M_LEFT45; //45$BEY<P$a$HF1$8;Q@*$K=$@5(B
                    diagonal_route_action[diagonal_action_id] = M_LEFTV90;
                    diagonal_route_action_times[diagonal_action_id] = 1.0;
                    diagonal_action_id++;

                    diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                    diagonal_route_action_times[diagonal_action_id] = 1;
                    diagonal_action_id++;

                    action_id += 2; //$B<!$N9TF0$^$G%9%-%C%W(B
                    break;
                }
            }
        }
	//action_id++;
    }
}


unsigned int calculate_fast_run_slalom_time_with_map(int x, int y)
{
    create_fast_run_slalom_map(x, y);
    //$B0z?t$N:BI8(Bx,y$B$K8~$+$C$F:GC;Av9T$9$k(B
    unsigned int total_runtime = 0;
	int action_id = 1;

	if (shortest_route_action_times[0] > 1.0) {
		total_runtime += (unsigned int) (((shortest_route_action_times[0]*SECTION+30.0)/ZENSHIN_MAX_SPEED) *1000); // cost milisecond runtime
	} else {	//1$B6h2h$OD>@~$rAv$k(B
		total_runtime += (unsigned int) (((shortest_route_action_times[0]*SECTION+30.0)/DAIKEI_ZENSHIN) *1000); // cost milisecond runtime
	}

	while(shortest_route_action[action_id] != M_REAR){	//$B%4!<%k$9$k$^$G7+$jJV$9(B
		switch(shortest_route_action[action_id])	//$B<!$K9T$/J}8~$rLa$jCM$H$9$k4X?t$r8F$V(B
		{
			case M_FRONT:	//$BD>@~$r$^$H$a$FAv$k$h$&$K$9$k(B
		        if (shortest_route_action[action_id+1] == M_REAR) {
                    total_runtime += (unsigned int) (((shortest_route_action_times[action_id]*SECTION)/DAIKEI_ZENSHIN) *1000); // cost milisecond runtime
		            break;
		        }

		        if (shortest_route_action_times[action_id] > 1.0) {
                    total_runtime += (unsigned int) (((shortest_route_action_times[action_id]*SECTION)/DAIKEI_ZENSHIN) *1000); // cost milisecond runtime
		            break;
		        } else {
                    total_runtime += (unsigned int) (((shortest_route_action_times[action_id]*SECTION)/DAIKEI_ZENSHIN) *1000); // cost milisecond runtime
		            break;		
			    }
			
			case M_RIGHT:
                total_runtime += DAIKEI_END_TIME; // cost milisecond runtime
			    break;
			
			case M_LEFT:
                total_runtime += DAIKEI_END_TIME; // cost milisecond runtime
                }
	       action_id++;
	}
    return total_runtime;
}


unsigned int calculate_max_fast_run_diagonal_time_with_map(int x, int y)
{
    fprintf(stderr, "start create fast recursion map\n");
    fflush(stderr);
    make_map_naname_recursion(x, y);
    fprintf(stderr, "start create fast diagonal slalom map\n");
    fflush(stderr);
    create_fast_run_diagonal_map(x, y);

    fprintf(stderr, "start convert fast slalom map to diagonal_route\n");
    fflush(stderr);
    convert_shortest_route_to_diagonal_route();

    //$B0z?t$N:BI8(Bx,y$B$K8~$+$C$F:GC;Av9T$9$k(B
    fprintf(stderr, "start calculate diagonal_route time\n");
    fflush(stderr);
    
    int action_id = 1;
    float prev_spd, next_spd;
    float diagonal_len, len;
    float max_spd;
    unsigned int total_runtime = 0;
    
    if (diagonal_route_action_times[0] > 1.0) {	 //$BD9$$D>@~$rA4B.$GAv$k(B
		total_runtime += (unsigned int) (((diagonal_route_action_times[0]*SECTION+30.0)/ZENSHIN_MAX_SPEED) *1000); // cost milisecond runtime
    } else {	//1$B6h2h$OD>@~$rAv$k(B
		total_runtime += (unsigned int) (((diagonal_route_action_times[0]*SECTION+30.0)/DAIKEI_ZENSHIN) *1000); // cost milisecond runtime
    }

    while(diagonal_route_action[action_id] != M_REAR){	//$B%4!<%k$9$k$^$G7+$jJV$9(B
        switch(diagonal_route_action[action_id]) {	//$B<!$K9T$/J}8~$rLa$jCM$H$9$k4X?t$r8F$V(B 
            case M_FRONT:
                len = diagonal_route_action_times[action_id]*SECTION;

                if (len > 0) {
                    if (diagonal_route_action_times[action_id] >= 2.5) {
                        total_runtime += (unsigned int) ((len/ZENSHIN_MAX_SPEED) *1000); // cost milisecond runtime
                    } else if (diagonal_route_action_times[action_id] > 1.0) {
                        total_runtime += (unsigned int) ((len/600) *1000); // cost milisecond runtime
                    } else {
                        total_runtime += (unsigned int) ((len/DAIKEI_ZENSHIN) *1000); // cost milisecond runtime
                    }
                }
                break;
                
            case M_RIGHT:
                total_runtime += DAIKEI_END_TIME; // cost milisecond runtime
                break;
                
            case M_LEFT:
                total_runtime += DAIKEI_END_TIME; // cost milisecond runtime
                break;
		
            case M_RIGHT45:		//$B1&(B45
                total_runtime += DAIKEI_45IN_END_TIME; // cost milisecond runtime
                break;

            case M_LEFT45:		//$B:8(B45
                total_runtime += DAIKEI_45IN_END_TIME; // cost milisecond runtime
                break;

            case M_RIGHTV90:		//$B1&(BV90
                if ((diagonal_route_action[action_id - 2] == M_LEFT135) && (diagonal_route_action[action_id + 1] != M_RIGHT45OUT) && (diagonal_route_action[action_id + 1] !=  M_LEFT45OUT)) { //135LEFT -> V90RIGHT ->  not 45OUT
                    total_runtime += DAIKEI_V90_END_TIME; // cost milisecond runtime
                    break;
                } else if ((diagonal_route_action[action_id + 2] == M_RIGHTV90 && diagonal_route_action[action_id + 3] == M_RIGHT45OUT) || (diagonal_route_action[action_id + 2] == M_LEFTV90 && diagonal_route_action[action_id + 3] == M_LEFT45OUT)) { //V90 -> V90 -> 45OUT
                    total_runtime += DAIKEI_V90_END_TIME; // cost milisecond runtime
                    break;
                } else if ((diagonal_route_action[action_id - 2] == M_LEFTV90 || (diagonal_route_action[action_id - 2] == M_RIGHTV90 && (int)diagonal_route_action_times[action_id - 1]%2 == 0)) && (diagonal_route_action[action_id + 1] == M_RIGHT45OUT)) { //V90 -> V90 -> 45OUT
                    total_runtime += DAIKEI_135OUT_END_TIME; // cost milisecond runtime
                    action_id++;
                    break;
                } else if (diagonal_route_action[action_id + 1] == M_RIGHT45OUT) { //135 -> V90 -> 45OUT
                    total_runtime += DAIKEI_135OUT_END_TIME; // cost milisecond runtime
                    action_id++;
                    break;
                }
                total_runtime += DAIKEI_V90_END_TIME; // cost milisecond runtime
                break;

            case M_LEFTV90:		//$B:8(BV90
                if ((diagonal_route_action[action_id - 2] == M_RIGHT135) && (diagonal_route_action[action_id + 1] != M_RIGHT45OUT) && (diagonal_route_action[action_id + 1] !=  M_LEFT45OUT)) { //135RIGHT -> V90LEFT ->  not 45OUT
                    total_runtime += DAIKEI_V90_END_TIME; // cost milisecond runtime
                    break;
                } else if ((diagonal_route_action[action_id + 2] == M_RIGHTV90 && diagonal_route_action[action_id + 3] == M_RIGHT45OUT) || (diagonal_route_action[action_id + 2] == M_LEFTV90 && diagonal_route_action[action_id + 3] == M_LEFT45OUT)) { //V90 -> V90 -> 45OUT
                    total_runtime += DAIKEI_V90_END_TIME; // cost milisecond runtime
                    break;
                } else if (((diagonal_route_action[action_id - 2] == M_LEFTV90 && (int)diagonal_route_action_times[action_id - 1]%2 == 0) || diagonal_route_action[action_id - 2] == M_RIGHTV90) && (diagonal_route_action[action_id + 1] == M_LEFT45OUT)) { //V90 -> V90 -> 45OUT
                    total_runtime += DAIKEI_135OUT_END_TIME; // cost milisecond runtime
                    action_id++;
                    break;
                } else if (diagonal_route_action[action_id + 1] == M_LEFT45OUT) { //135 -> V90 -> 45OUT
                    total_runtime += DAIKEI_135OUT_END_TIME; // cost milisecond runtime
                    action_id++;
                    break;
                }
                total_runtime += DAIKEI_V90_END_TIME; // cost milisecond runtime
                break;

            case M_RIGHT135:		//$B1&(B135
                total_runtime += DAIKEI_135IN_END_TIME; // cost milisecond runtime
                break;

            case M_LEFT135:		//$B:8(B135
                total_runtime += DAIKEI_135IN_END_TIME; // cost milisecond runtime
                break;

            case M_RIGHT180:	    //$B1&(B180
                total_runtime += DAIKEI_OOMAWARI180_END_TIME; // cost milisecond runtime
                total_runtime += (unsigned int) (((HALF_SECTION/2)/DAIKEI_ZENSHIN) *1000); // cost milisecond runtime
                break;

            case M_LEFT180:     	//$B:8(B180
                total_runtime += DAIKEI_OOMAWARI180_END_TIME; // cost milisecond runtime
                total_runtime += (unsigned int) (((HALF_SECTION/2)/DAIKEI_ZENSHIN) *1000); // cost milisecond runtime
                break;

            case M_RIGHT45OUT:	 //$B1&(B45$B=P8}(B
                total_runtime += DAIKEI_45OUT_END_TIME; // cost milisecond runtime
                break;

            case M_LEFT45OUT:	     //$B:8(B45$B=P8}(B
                total_runtime += DAIKEI_45OUT_END_TIME; // cost milisecond runtime
                break;

            case M_FRONT_DIAGONAL: //$B<P$aA0?J(B
                diagonal_len = diagonal_route_action_times[action_id];
                max_spd = DAIKEI_ZENSHIN;
                diagonal_len -= 1.0;
                
                if(diagonal_len > 0) {
                    max_spd = 700.0;
	                if (diagonal_len >= 4.0) {
	                    max_spd = ZENSHIN_MAX_SPEED;
	                } else if (diagonal_len >= 2.0) {
	                    max_spd = 800.0;
	                }

	                len = diagonal_len*(2.0*DIAGONAL_QUARTER_SECTION);
                    total_runtime += (unsigned int) ((len/max_spd) *1000); // cost milisecond runtime
                    break;
                }
        }
	action_id++;
    }
    return total_runtime;
}



int get_nextdir_fast_run(int x, int y, int mask, t_direction *dir)	
{
	//$B%4!<%k:BI8(Bx,y$B$K8~$+$&>l9g!":#$I$A$i$K9T$/$Y$-$+$rH=CG$9$k!#(B
	//$BC5:w!":GC;$N@Z$jBX$($N$?$a$N(Bmask$B$r;XDj!"(Bdir$B$OJ}3Q$r<($9(B
	int little,priority,tmp_priority;								//$B:G>.$NCM$rC5$9$?$a$K;HMQ$9$kJQ?t(B
 
	make_map(x,y,mask);												//$BJb?t(BMap$B@8@.(B
	//make_fast_run_map_with_run_time(x,y,mask);												//$BJb?t(BMap$B@8@.(B
	little = 255;													//$B:G>.Jb?t$r(B255$BJb(B(map$B$,(Bunsigned char$B7?$J$N$G(B)$B$K@_Dj(B	

	priority = 0;													//$BM%@hEY$N=i4|CM$O(B0
	
    //mask$B$N0UL#$O(Bstatic_parameter.h$B$r;2>H(B
	if( (wall[mypos.x][mypos.y].north & mask) == NOWALL)			//$BKL$KJI$,$J$1$l$P(B
	{
		tmp_priority = get_priority(mypos.x, mypos.y + 1, north);	//$BM%@hEY$r;;=P(B
		if(map[mypos.x][mypos.y+1] < little)						//$B0lHVJb?t$,>.$5$$J}8~$r8+$D$1$k(B
		{
			little = map[mypos.x][mypos.y+1];						//$B$R$H$^$:KL$,Jb?t$,>.$5$$;v$K$9$k(B
			*dir = north;											//$BJ}8~$rJ]B8(B
			priority = tmp_priority;								//$BM%@hEY$rJ]B8(B
		}
		else if(map[mypos.x][mypos.y+1] == little)					//$BJb?t$,F1$8>l9g$OM%@hEY$+$iH=CG$9$k(B
		{
			if(priority < tmp_priority )							//$BM%@hEY$rI>2A(B
			{
				*dir = north;										//$BJ}8~$r99?7(B
				priority = tmp_priority;							//$BM%@hEY$rJ]B8(B
			}
		}
	}
	
	if( (wall[mypos.x][mypos.y].east & mask) == NOWALL)				//$BEl$KJI$,$J$1$l$P(B
	{
		tmp_priority = get_priority(mypos.x + 1, mypos.y, east);	//$BM%@hEY$r;;=P(B
		if(map[mypos.x + 1][mypos.y] < little)						//$B0lHVJb?t$,>.$5$$J}8~$r8+$D$1$k(B
		{
			little = map[mypos.x+1][mypos.y];						//$B$R$H$^$:El$,Jb?t$,>.$5$$;v$K$9$k(B
			*dir = east;											//$BJ}8~$rJ]B8(B
			priority = tmp_priority;								//$BM%@hEY$rJ]B8(B
		}
		else if(map[mypos.x + 1][mypos.y] == little)				//$BJb?t$,F1$8>l9g!"M%@hEY$+$iH=CG(B
		{
			if(priority < tmp_priority)								//$BM%@hEY$rI>2A(B
			{
				*dir = east;										//$BJ}8~$rJ]B8(B
				priority = tmp_priority;							//$BM%@hEY$rJ]B8(B
			}
		}
	}
	
	if( (wall[mypos.x][mypos.y].south & mask) == NOWALL)			//$BFn$KJI$,$J$1$l$P(B
	{
		tmp_priority = get_priority(mypos.x, mypos.y - 1, south);	//$BM%@hEY$r;;=P(B
		if(map[mypos.x][mypos.y - 1] < little)						//$B0lHVJb?t$,>.$5$$J}8~$r8+$D$1$k(B
		{
			little = map[mypos.x][mypos.y-1];						//$B$R$H$^$:Fn$,Jb?t$,>.$5$$;v$K$9$k(B
			*dir = south;											//$BJ}8~$rJ]B8(B
			priority = tmp_priority;								//$BM%@hEY$rJ]B8(B
		}
		else if(map[mypos.x][mypos.y - 1] == little)				//$BJb?t$,F1$8>l9g!"M%@hEY$GI>2A(B
		{
			if(priority < tmp_priority)								//$BM%@hEY$rI>2A(B
			{
				*dir = south;										//$BJ}8~$rJ]B8(B
				priority = tmp_priority;							//$BM%@hEY$rJ]B8(B
			}
		}
	}
	
	if( (wall[mypos.x][mypos.y].west & mask) == NOWALL)				//$B@>$KJI$,$J$1$l$P(B
	{
		tmp_priority = get_priority(mypos.x - 1, mypos.y, west);	//$BM%@hEY$r;;=P(B
		if(map[mypos.x-1][mypos.y] < little)						//$B0lHVJb?t$,>.$5$$J}8~$r8+$D$1$k(B
		{
			little = map[mypos.x-1][mypos.y];						//$B@>$,Jb?t$,>.$5$$(B
			*dir = west;											//$BJ}8~$rJ]B8(B
			priority = tmp_priority;								//$BM%@hEY$rJ]B8(B
		}
		else if(map[mypos.x - 1][mypos.y] == little)				//$BJb?t$,F1$8>l9g!"M%@hEY$GI>2A(B
		{
			*dir = west;											//$BJ}8~$rJ]B8(B
			priority = tmp_priority;								//$BM%@hEY$rJ]B8(B
		}
	}

    if(little == 255) {
	unable_to_find_path_to_goal = 1;
	return stop;
    }

	return ( (int)( ( 4 + *dir - mypos.dir) % 4 ) );				//$B$I$C$A$K8~$+$&$Y$-$+$rJV$9!#(B
}



void log(char* text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}

int main(int argc, char* argv[]) {
    log("Running...");
    API_setColor(0, 0, 'G');
    API_setText(0, 0, "abc");

    //set_wall_whole_map();
    ///*
	mypos.x = mypos.y = 0;							//$B:BI8$r=i4|2=(B
	mypos.dir = north;								//$BJ}3Q$r=i4|2=(B
    set_wall(mypos.x,mypos.y);					//$BJI$r%;%C%H(B

    API_turnLeft();
    API_turnLeft();
	mypos.x = mypos.y = 0;							//$B:BI8$r=i4|2=(B
	mypos.dir = south;								//$BJ}3Q$r=i4|2=(B
    set_wall(mypos.x,mypos.y);					//$BJI$r%;%C%H(B

    API_turnLeft();
    API_turnLeft();
	mypos.dir = north;								//$BJ}3Q$r=i4|2=(B

	fast_full_search_adachi_slalom(15,8, 15,8);
	fast_full_search_adachi_slalom(7,0, 15,8);
	fast_full_search_adachi_slalom(15,8, 7,0);
	fast_full_search_adachi_slalom(7,0,7,0);

    API_turnLeft();
    API_turnLeft();
    //*/
	mypos.x = mypos.y = 0;							//$B:BI8$r=i4|2=(B
	mypos.dir = north;								//$BJ}3Q$r=i4|2=(B
    API_clearAllColor();

    //get_nextdir(GOAL_X, GOAL_Y,MASK_SECOND,&mypos.dir);

    // $BJI$HCl$N%^%C%W$r:n@.(B
    for(int j = 0; j < MAZESIZE_Y; j++)						//$BLBO)$NBg$-$5J,%k!<%W(B(x$B:BI8(B)
    {
    	for(int i = 0; i < MAZESIZE_X; i++)					//$BLBO)$NBg$-$5J,%k!<%W(B(y$B:BI8(B)
    	{
            if((wall[i][j].north & MASK_SEARCH) == WALL) {
                wall_naname[(i*2)+2][(j*2)+2] = WALL;
                wall_naname[(i*2)+1][(j*2)+2] = WALL;
                wall_naname[(i*2)+0][(j*2)+2] = WALL;
            } 
            if((wall[i][j].south & MASK_SEARCH) == WALL) {
                wall_naname[(i*2)+2][(j*2)+0] = WALL;
                wall_naname[(i*2)+1][(j*2)+0] = WALL;
                wall_naname[(i*2)+0][(j*2)+0] = WALL;
            } 
            if((wall[i][j].east & MASK_SEARCH) == WALL) {
                wall_naname[(i*2)+2][(j*2)+2] = WALL;
                wall_naname[(i*2)+2][(j*2)+1] = WALL;
                wall_naname[(i*2)+2][(j*2)+0] = WALL;
            } 
            if((wall[i][j].west & MASK_SEARCH) == WALL) {
                wall_naname[(i*2)+0][(j*2)+2] = WALL;
                wall_naname[(i*2)+0][(j*2)+1] = WALL;
                wall_naname[(i*2)+0][(j*2)+0] = WALL;
            } 
        }
    }
    //wall_naname[(8*2)+2][(6*2)+2] = WALL;
    //wall_naname[(8*2)+1][(6*2)+2] = WALL;
    //wall_naname[(8*2)+0][(6*2)+2] = WALL;

    // $BJI$HCl$rI=<((B
    for(int j = MAZESIZE_Y*2; j >= 0; j--)						//$BLBO)$NBg$-$5J,%k!<%W(B(x$B:BI8(B)
    {
    	for(int i = 0; i < MAZESIZE_X*2+1; i++)					//$BLBO)$NBg$-$5J,%k!<%W(B(y$B:BI8(B)
    	{
            if(wall_naname[i][j] == WALL) {
                fprintf(stderr, "# ");
                fflush(stderr);
            } else if(wall_naname[i][j] == NOWALL) {
                fprintf(stderr, "0 ");
                fflush(stderr);
            } else {
                fprintf(stderr, "! ");
                fflush(stderr);
            }
        }
        fprintf(stderr, "\n");
        fflush(stderr);
    }


    
    // recusion$B$G:GB.<P$a%3%9%H%^%C%W:n@.(B
	//init_map_naname(GOAL_X,GOAL_Y);											//Map$B$r=i4|2=$9$k(B
    //prioritize_straight_cost_recursion(GOAL_X*2+1, GOAL_Y*2+1, north, north, fast_straight_cost);
    //prioritize_straight_cost_recursion(GOAL_X*2+1, GOAL_Y*2+1, south, south, fast_straight_cost);
    //make_map_naname_recursion(GOAL_X, GOAL_Y);
    //create_fast_run_diagonal_map(GOAL_X, GOAL_Y);
    unsigned int timer;
    recusion_weight =10;
    color = 'G';
    timer = calculate_max_fast_run_diagonal_time_with_map(GOAL_X, GOAL_Y);
    fprintf(stderr, "\n\nfirst total runtime: %u\n\n", timer);
    fflush(stderr);

	mypos.x = mypos.y = 0;							//$B:BI8$r=i4|2=(B
	mypos.dir = north;								//$BJ}3Q$r=i4|2=(B
    recusion_weight =20;
    color = 'B';
    timer = calculate_max_fast_run_diagonal_time_with_map(GOAL_X, GOAL_Y);
    fprintf(stderr, "\n\nsecond total runtime: %u\n\n", timer);
    fflush(stderr);

	mypos.x = mypos.y = 0;							//$B:BI8$r=i4|2=(B
	mypos.dir = north;								//$BJ}3Q$r=i4|2=(B
    recusion_weight =30;
    color = 'C';
    timer = calculate_max_fast_run_diagonal_time_with_map(GOAL_X, GOAL_Y);
    fprintf(stderr, "\n\nthird total runtime: %u\n\n", timer);
    fflush(stderr);

	mypos.x = mypos.y = 0;							//$B:BI8$r=i4|2=(B
	mypos.dir = north;								//$BJ}3Q$r=i4|2=(B
    recusion_weight =40;
    color = 'y';
    timer = calculate_max_fast_run_diagonal_time_with_map(GOAL_X, GOAL_Y);
    fprintf(stderr, "\n\nfourth total runtime: %u\n\n", timer);
    fflush(stderr);

	mypos.x = mypos.y = 0;							//$B:BI8$r=i4|2=(B
	mypos.dir = north;								//$BJ}3Q$r=i4|2=(B
    recusion_weight =50;
    color = 'o';
    timer = calculate_max_fast_run_diagonal_time_with_map(GOAL_X, GOAL_Y);
    fprintf(stderr, "\n\nfifth total runtime: %u\n\n", timer);
    fflush(stderr);
    /*
    // $B%3%9%H%^%C%W$rI=<((B
    char str[5];
    unsigned short weight;
    for(int j = 15; j >= 0; j--)						//$BLBO)$NBg$-$5J,%k!<%W(B(x$B:BI8(B)
    {
    	for(int i = 0; i < MAZESIZE_X; i++)					//$BLBO)$NBg$-$5J,%k!<%W(B(y$B:BI8(B)
    	{
            weight = map_naname[i*2+1][j*2+1];       
            fprintf(stderr, "%u   ", weight);
            fflush(stderr);
            sprintf(str, "%u", weight);
            API_setText(i, j, str);
        }
        fprintf(stderr, "\n");
        fprintf(stderr, "\n");
        fflush(stderr);
    }
    //*/
    /*
    // 32x32$B%3%9%H%^%C%W$rI=<((B
    char str[5];
    unsigned short weight;
    for(int j = MAX_Y; j >= 0; j--)						//$BLBO)$NBg$-$5J,%k!<%W(B(x$B:BI8(B)
    {
    	for(int i = 0; i < MAX_X+1; i++)					//$BLBO)$NBg$-$5J,%k!<%W(B(y$B:BI8(B)
    	{
            weight = map_naname[i][j];       
            fprintf(stderr, "%u   ", weight);
            fflush(stderr);
            sprintf(str, "%u", weight);
            //API_setText(i, j, str);
        }
        fprintf(stderr, "\n");
        fprintf(stderr, "\n");
        fflush(stderr);
    }
    */


    /*
	mypos.x = mypos.y = 7;							//$B:BI8$r=i4|2=(B
	mypos.dir = north;								//$BJ}3Q$r=i4|2=(B
    t_direction fast_dir = get_nextdir(0, 0,MASK_SECOND,&mypos.dir);
	mypos.dir = fast_dir;								//$BJ}3Q$r=i4|2=(B
    */
}
