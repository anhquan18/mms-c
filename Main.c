#include <stdio.h>
#include "API.h"


#define MAZESIZE_X	(16)		//$BLBO)$NBg$-$5(B(MAZESIZE_X * MAZESIZE_Y)$BLBO)(B
#define MAZESIZE_Y	(16)		//$BLBO)$NBg$-$5(B(MAZESIZE_X * MAZESIZE_Y)$BLBO)(B
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
unsigned char	map[MAZESIZE_X][MAZESIZE_Y];	//$BJb?t%^%C%W(B
//unsigned int		timer;							//1mS$B$4$H$K%+%&%s%H%"%C%W$5$l$kJQ?t(B.



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
	                    if(unable_to_find_path_to_goal == 1) {
	                        unable_to_find_path_to_goal = 0;
	                        continue;
	                    }
			    
	                    //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);	//$B0l6h2h?J$`(B
                        API_moveForward();
	                    break;
	                
	                case right:
	                    //rotate(right,1);									//$B1&$K6J$,$C$F(B
	                    //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);	//$BH>6h2h?J$`(B
                        API_turnRight();
                        API_moveForward();
	                    break;
	                
	                case left:
	                    //rotate(left,1);										//$B:8$K6J$,$C$F(B
	                    //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);	//$BH>6h2h?J$`(B
                        API_turnLeft();
                        API_moveForward();
	                    break;
	                
	                case rear:
			    
	                    //rotate(left,2);										//180$B%?!<%s(B
	                    //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);	//$BH>6h2h?J$`(B
                        API_turnLeft();
                        API_turnLeft();
                        API_moveForward();
	                    break;
			case stop:
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
	            
	            while(((mypos.x != gx) || (mypos.y != gy)) && (!no_goal_exit_loop_flag)){							//$B%4!<%k$9$k$^$G7+$jJV$9(B
                    fprintf(stderr, "current x: %d, y: %d | goal:%d, %d \n", mypos.x, mypos.y, gx, gy);
                    fflush(stderr);
	                	if(mypos.x<0 || mypos.x>15 ) 	{
                            return 1;
				}
				if(mypos.y<0 || mypos.y>15 ) {
                    return 1;
				}    
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



void make_fast_run_map(int x, int y, int mask)	//$BJb?t%^%C%W$r:n@.$9$k(B
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



int get_nextdir_fast_run(int x, int y, int mask, t_direction *dir)	
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




void log(char* text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}

int main(int argc, char* argv[]) {
    log("Running...");
    API_setColor(0, 0, 'G');
    API_setText(0, 0, "abc");
	mypos.x = mypos.y = 0;							//$B:BI8$r=i4|2=(B
    set_wall(mypos.x,mypos.y);					//$BJI$r%;%C%H(B

    API_turnLeft();
    API_turnLeft();
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
	mypos.dir = north;								//$BJ}3Q$r=i4|2=(B
    API_clearAllColor();
    get_nextdir(GOAL_X, GOAL_Y,MASK_SECOND,&mypos.dir);

    /*
	mypos.x = mypos.y = 7;							//$B:BI8$r=i4|2=(B
	mypos.dir = north;								//$BJ}3Q$r=i4|2=(B
    t_direction fast_dir = get_nextdir(0, 0,MASK_SECOND,&mypos.dir);
	mypos.dir = fast_dir;								//$BJ}3Q$r=i4|2=(B
    */
}
