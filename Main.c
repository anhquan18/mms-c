#include <stdio.h>
#include "API.h"


#define MAZESIZE_X	(16)		//迷路の大きさ(MAZESIZE_X * MAZESIZE_Y)迷路
#define MAZESIZE_Y	(16)		//迷路の大きさ(MAZESIZE_X * MAZESIZE_Y)迷路
#define MAX_X	(MAZESIZE_X*2)		//迷路の大きさ(MAZESIZE_X * MAZESIZE_Y)迷路
#define MAX_Y	(MAZESIZE_Y*2)		//迷路の大きさ(MAZESIZE_X * MAZESIZE_Y)迷路
#define GOAL_X	7							//ゴール座標(x)
#define GOAL_Y	7							//ゴール座標(y)

#define PI (3.141592653589793)	//円周率

#define DIAGONAL_QUARTER_SECTION (63.639)	//1/4区画の斜め距離
#define QUARTER_SECTION	(45.0)	//1/4区画の距離
#define HALF_SECTION	(90.0)	//半区画の距離
#define SECTION		(180.0)		//一区画の距離

#define UNKNOWN	2				//壁があるかないか判らない状態の場合の値
#define NOWALL	0				//壁がないばあいの値
#define WALL	1				//壁がある場合の値
#define VWALL	3				//仮想壁の値(未使用)

#define MASK_SEARCH	0x01		//探索走行用マスク値.壁情報とこの値のAND値が０（NOWALL）なら壁なしor未探索区間
#define MASK_SECOND	0x03		//最短走行用マスク値.壁情報とこの値のAND値が０（NOWALL）なら壁なし


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

//地図の向き
#define M_FRONT (0)		//前
#define M_RIGHT (1)		//右
#define M_REAR (2)			//後
#define M_LEFT (3)			//左
#define M_RIGHT_N (19)		//右90大回り
#define M_LEFT_N (20)		//左90大回り
#define M_RIGHT_F (17)		//右90大回り
#define M_LEFT_F (18)		//左90大回り
#define M_RIGHT45 (4)		//右45
#define M_LEFT45 (5)		//左45
#define M_RIGHTV90 (6)		//右V90
#define M_LEFTV90 (7)		//左V90
#define M_RIGHT135 (8)		//右135
#define M_LEFT135 (9)		//左135
#define M_RIGHT180 (10)	        //右180
#define M_LEFT180 (11)		//左180
#define M_RIGHT180_F (15)       //右180大回り
#define M_LEFT180_F (16)	//左180大回り
#define M_RIGHT45OUT (12)	//右45出口
#define M_LEFT45OUT (13)	//左45出口
#define M_FRONT_DIAGONAL (14) //斜め前進



typedef enum
{
	false = 0,	//偽
	true = 1,	//真
}t_bool;		//真偽値を取り扱う列挙型

typedef enum
{
	front=0,		//前
	right=1,		//右
	rear=2,			//後
	left=3,			//左
	stop=5,
	unknown,		//方向不明
}t_local_dir;	//自分から見た方向を示す列挙型

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
	unsigned char north:2;	//北の壁情報
	unsigned char east:2;	//東の壁情報
	unsigned char south:2;	//南の壁情報
	unsigned char west:2;	//西の壁情報
}t_wall;					//壁情報を格納する構造体(ビットフィールド)



short already_passed_boolean_map[256] = {0};
short unable_to_find_path_to_goal = 0;
t_position		mypos;							//自己座標
t_wall			wall[MAZESIZE_X][MAZESIZE_Y];	//壁の情報を格納する構造体配列
unsigned char  wall_naname[MAZESIZE_X*2+1][MAZESIZE_Y*2+1];	//壁の情報を格納する構造体配列
unsigned char	map[MAZESIZE_X][MAZESIZE_Y];	//歩数マップ
unsigned short	map_naname[MAZESIZE_X*2+1][MAZESIZE_Y*2+1];	//歩数マップ
short shortest_route_action[256]; // 進む・左・右などの向きで前進・回転を表現
float shortest_route_action_times[256]; // 数字で当てはまる行動(前進・回転)の回数を表現
short diagonal_route_action[256]; // 進む・左・右などの向きで前進・回転を表現
float diagonal_route_action_times[256]; // 数字で当てはまる斜め行動(前進・回転)の回数を表現
unsigned short fast_straight_cost = (unsigned short) (ZENSHIN_END_TIME*1000);
unsigned short fast_naname_cost = (unsigned short) (NANAME_END_TIME*1000);
char color = 'G';
unsigned short recusion_weight = 0;



void init_map(int x, int y)
{
//迷路の歩数Mapを初期化する。全体を0xff、引数の座標x,yは0で初期化する
	int i,j;
	for(i = 0; i < MAZESIZE_X; i++)		//迷路の大きさ分ループ(x座標)
	{
		for(j = 0; j < MAZESIZE_Y; j++)	//迷路の大きさ分ループ(y座標)
		{
			map[i][j] = 255;			//すべて255で埋める
            API_setText(i, j, "255");
		}
	}
	map[x][y] = 0;						//ゴール座標の歩数を０に設定
    API_setText(x, y, "0");
}


void init_map_naname(int x, int y)
{
//迷路の歩数Mapを初期化する。全体を0xff、引数の座標x,yは0で初期化する
	int i,j;
	for(i = 0; i < MAZESIZE_X*2+1; i++)		//迷路の大きさ分ループ(x座標)
	{
		for(j = 0; j < MAZESIZE_Y*2+1; j++)	//迷路の大きさ分ループ(y座標)
		{
			map_naname[i][j] = 59999;			//すべて999で埋める
		}
	}
	map_naname[x*2+1][y*2+1] = 0;						//ゴール座標の歩数を０に設定
}



void make_map(int x, int y, int mask)	//歩数マップを作成する
{
//座標x,yをゴールとした歩数Mapを作成する。
//maskの値(MASK_SEARCH or MASK_SECOND)によって、
//探索用の歩数Mapを作るか、最短走行の歩数Mapを作るかが切り替わる\

	int i,j;
	t_bool change_flag;										//Map作成終了を見極めるためのフラグ
    char str[5];

    API_clearAllText();
	init_map(x,y);											//Mapを初期化する
	do
	{
		change_flag = false;								//変更がなかった場合にはループを抜ける
		for(i = 0; i < MAZESIZE_X; i++)						//迷路の大きさ分ループ(x座標)
		{
			for(j = 0; j < MAZESIZE_Y; j++)					//迷路の大きさ分ループ(y座標)
			{
				if(map[i][j] == 255)						//255の場合は次へ
				{
					continue;
				}
				
				if(j < MAZESIZE_Y-1)						//範囲チェック
				{
					if( (wall[i][j].north & mask) == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
					{
						if(map[i][j+1] == 255)				//まだ値が入っていなければ
						{
							map[i][j+1] = map[i][j] + 1;	//値を代入
                            sprintf(str, "%u", map[i][j+1]);
                            API_setText(i, j+1, str);
							change_flag = true;				//値が更新されたことを示す
						}
					}
				}
			
				if(i < MAZESIZE_X-1)						//範囲チェック
				{
					if( (wall[i][j].east & mask) == NOWALL)	//壁がなければ
					{
						if(map[i+1][j] == 255)				//値が入っていなければ
						{
							map[i+1][j] = map[i][j] + 1;	//値を代入
                            sprintf(str, "%u", map[i+1][j]);
                            API_setText(i+1, j, str);
							change_flag = true;				//値が更新されたことを示す
						}
					}
				}
			
				if(j > 0)									//範囲チェック
				{
					if( (wall[i][j].south & mask) == NOWALL)//壁がなければ
					{
						if(map[i][j-1] == 255)				//値が入っていなければ
						{
							map[i][j-1] = map[i][j] + 1;	//値を代入
                            sprintf(str, "%u", map[i][j-1]);
                            API_setText(i, j-1, str);
							change_flag = true;				//値が更新されたことを示す
						}
					}
				}
			
				if(i > 0)									//範囲チェック
				{
					if( (wall[i][j].west & mask) == NOWALL)	//壁がなければ
					{
						if(map[i-1][j] == 255)				//値が入っていなければ
						{
							map[i-1][j] = map[i][j] + 1;	//値を代入	
                            sprintf(str, "%u", map[i-1][j]);
                            API_setText(i-1, j, str);
							change_flag = true;				//値が更新されたことを示す
						}
						
					}
					
				}
				
			}
			
		}
		
	}while(change_flag == true);	//全体を作り終わるまで待つ
}



void check_and_set_wall(int x, int y, int is_wall, char direction)
{
    if(is_wall) {
        API_setWall(x,y,direction);
    } 
}


void set_wall(int x, int y)	//壁情報を記録
{
//引数の座標x,yに壁情報を書き込む
	int n_write,s_write,e_write,w_write;
	
	
	//自分の方向に応じて書き込むデータを生成
	//CONV_SEN2WALL()はmacro.hを参照
	switch(mypos.dir){
		case north:	//北を向いている時
			n_write = API_wallFront();	//前壁の有無を判断
			e_write = API_wallRight();  //右壁の有無を判断
			w_write = API_wallLeft();   //左壁の有無を判断
			s_write = NOWALL;		    //後ろは必ず壁がない
			break;
			
		case east:	//東を向いているとき
			e_write = API_wallFront();	//前壁の有無を判断
			s_write = API_wallRight();  //右壁の有無を判断
			n_write = API_wallLeft();   //左壁の有無を判断
			w_write = NOWALL;											//後ろは必ず壁がない
			break;
			
		case south:	//南を向いているとき
			s_write = API_wallFront();	//前壁の有無を判断
			w_write = API_wallRight();  //右壁の有無を判断
			e_write = API_wallLeft();   //左壁の有無を判断
			n_write = NOWALL;											//後ろは必ず壁がない
			break;
			
		case west:	//西を向いているとき
			w_write = API_wallFront();		//前壁の有無を判断
			n_write = API_wallRight();		//右壁の有無を判断
			s_write = API_wallLeft();		//左壁の有無を判断
			e_write = NOWALL;			NOWALL;											//後ろは必ず壁がない
			break;
	}
	
	wall[x][y].north = n_write;	//実際に壁情報を書き込み
    check_and_set_wall(x,y,n_write,'n');	

    wall[x][y].south = s_write;	//実際に壁情報を書き込み
    check_and_set_wall(x,y,s_write,'s');	

	wall[x][y].east  = e_write;	//実際に壁情報を書き込み
    check_and_set_wall(x,y,e_write,'e');	

	wall[x][y].west  = w_write;	//実際に壁情報を書き込み
    check_and_set_wall(x,y,w_write,'w');	
	
	if(y < MAZESIZE_Y-1)				//範囲チェック
	{
		wall[x][y+1].south = n_write;	//反対側から見た壁を書き込み
        //check_and_set_wall(x, y+1, n_write,'s');	
	}
	
	if(x < MAZESIZE_X-1)				//範囲チェック
	{
		wall[x+1][y].west = e_write;	//反対側から見た壁を書き込み
        //check_and_set_wall(x+1, y, e_write,'w');	
	}
	
	if(y > 0)							//範囲チェック
	{
		wall[x][y-1].north = s_write;	//反対側から見た壁を書き込み
        //check_and_set_wall(x, y-1, s_write,'n');	
	}
	
	if(x > 0)							//範囲チェック
	{
		wall[x-1][y].east = w_write;	//反対側から見た壁を書き込み
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
            if(y < MAX_Y)	// 前進動作
            {
            	if (wall_naname[x][y+1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    if(prev_dir != current_dir) { // 向きが変わった場合、回転の重みから直線の重みにリセット
                        weight = fast_straight_cost;
                    }
            		if((map_naname[x][y] + weight) <= map_naname[x][y+1])				//まだ値が入っていなければ
            		{
            			map_naname[x][y+1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x, y+1, north, north, weight);
                    }
            	}
            }

            if((y < MAX_Y) && (x < MAX_X))	// 右45度回転
            {
            	if (wall_naname[x+1][y+1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_45IN_END_TIME; // 向きを45度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y+1])				//まだ値が入っていなければ
            		{
            			map_naname[x+1][y+1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x+1, y+1, north, north_east, weight);
                    }
            	}
            }

            if((y < MAX_Y) && (x > 0))	// 左45度回転
            {
            	if (wall_naname[x-1][y+1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_45IN_END_TIME; // 向きを45度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y+1])				//まだ値が入っていなければ
            		{
            			map_naname[x-1][y+1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x-1, y+1, north, north_west, weight);
                    }
            	}
            }

            if(x < MAX_X)	// 右90度回転
            {
            	if (wall_naname[x+1][y] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_END_TIME; // 向きを90度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y])				//まだ値が入っていなければ
            		{
            			map_naname[x+1][y] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x+1, y, north, east, weight);
                    }
            	}
            }

            if(x > 0)	// 左90度回転
            {
            	if (wall_naname[x-1][y] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_END_TIME; // 向きを90度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y])				//まだ値が入っていなければ
            		{
            			map_naname[x-1][y] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x-1, y, north, west, weight);
                    }
            	}
            }

            break;

        case north_east:
            if((y < MAX_Y) && (x < MAX_X))						//範囲チェック
            {
            	if (wall_naname[x+1][y+1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    if(prev_dir != current_dir) { // 向きが変わった場合、回転の重みから直線の重みにリセット
                        weight = fast_naname_cost;
                    }
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y+1])				//まだ値が入っていなければ
            		{
            			map_naname[x+1][y+1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x+1, y+1, north_east, north_east, weight);
                    }
            	}
            }
            if(x < MAX_X)	// 右45度回転
            {
            	if (wall_naname[x+1][y] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_45OUT_END_TIME; // 向きを45度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y])				//まだ値が入っていなければ
            		{
            			map_naname[x+1][y] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x+1, y, north_east, east, weight);
                    }
            	}
            }

            if(y < MAX_Y)	// 左45度回転
            {
            	if (wall_naname[x][y+1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_45OUT_END_TIME; // 向きを45度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x][y+1])				//まだ値が入っていなければ
            		{
            			map_naname[x][y+1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x, y+1, north_east, north, weight);
                    }
            	}
            }

            if((x < MAX_X) && (y > 0))	// 右90度回転
            {
            	if (wall_naname[x+1][y-1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_END_TIME; // 向きを90度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y-1])				//まだ値が入っていなければ
            		{
            			map_naname[x+1][y-1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x+1, y-1, north_east, south_east, weight);
                    }
            	}
            }

            if((x > 0) && (y < MAX_Y))	// 左90度回転
            {
            	if (wall_naname[x-1][y+1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_END_TIME; // 向きを90度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y+1])				//まだ値が入っていなければ
            		{
            			map_naname[x-1][y+1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x-1, y+1, north_east, north_west, weight);
                    }
            	}
            }
            break;

        case north_west:
            if((y < MAX_Y) && (x > 0))						//範囲チェック
            {
            	if (wall_naname[x-1][y+1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    if(prev_dir != current_dir) { // 向きが変わった場合、回転の重みから直線の重みにリセット
                        weight = fast_naname_cost;
                    }
            		if ((map_naname[x][y] + weight) <= map_naname[x-1][y+1])				//まだ値が入っていなければ
            		{
            			map_naname[x-1][y+1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x-1, y+1, north_west, north_west, weight);
                    }
            	}
            }

            if(y < MAX_Y)	// 右45度回転
            {
            	if (wall_naname[x][y+1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_45OUT_END_TIME; // 向きを45度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x][y+1])				//まだ値が入っていなければ
            		{
            			map_naname[x][y+1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x, y+1, north_west, north, weight);
                    }
            	}
            }

            if(x > 0)	// 左45度回転
            {
            	if (wall_naname[x-1][y] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_45OUT_END_TIME; // 向きを45度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y])				//まだ値が入っていなければ
            		{
            			map_naname[x-1][y] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x-1, y, north_west, west, weight);
                    }
            	}
            }

            if((x < MAX_X) && (y < MAX_Y))	// 右90度回転
            {
            	if (wall_naname[x+1][y+1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_END_TIME; // 向きを90度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y+1])				//まだ値が入っていなければ
            		{
            			map_naname[x+1][y+1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x+1, y+1, north_west, north_east, weight);
                    }
            	}
            }

            if((x > 0) && (y > 0))	// 左90度回転
            {
            	if (wall_naname[x-1][y-1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_END_TIME; // 向きを90度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y-1])				//まだ値が入っていなければ
            		{
            			map_naname[x-1][y-1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x-1, y-1, north_west, south_west, weight);
                    }
            	}
            }
            break;

        case south:
            if(y > 0) 						//範囲チェック
            {
            	if (wall_naname[x][y-1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    if(prev_dir != current_dir) { // 向きが変わった場合、回転の重みから直線の重みにリセット
                        weight = fast_straight_cost;
                    }
            		if ((map_naname[x][y] + weight) <= map_naname[x][y-1])				//まだ値が入っていなければ
            		{
            			map_naname[x][y-1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x, y-1, south, south, weight);
                    }
            	}
            }

            if((y > 0) && (x > 0))	// 右45度回転
            {
            	if (wall_naname[x-1][y-1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_45IN_END_TIME; // 向きを45度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y-1])				//まだ値が入っていなければ
            		{
            			map_naname[x-1][y-1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x-1, y-1, south, south_west, weight);
                    }
            	}
            }

            if((y > 0) && (x < MAX_X))	// 左45度回転
            {
            	if (wall_naname[x+1][y-1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_45IN_END_TIME; // 向きを45度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y-1])				//まだ値が入っていなければ
            		{
            			map_naname[x+1][y-1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x+1, y-1, south, south_east, weight);
                    }
            	}
            }

            if(x > 0)	// 右90度回転
            {
            	if (wall_naname[x-1][y] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_END_TIME; // 向きを90度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y])				//まだ値が入っていなければ
            		{
            			map_naname[x-1][y] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x-1, y, south, west, weight);
                    }
            	}
            }

            if(x < MAX_X)	// 左90度回転
            {
            	if (wall_naname[x+1][y] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_END_TIME; // 向きを90度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y])				//まだ値が入っていなければ
            		{
            			map_naname[x+1][y] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x+1, y, south, east, weight);
                    }
            	}
            }
            break;

        case south_east:
            if((y > 0) && (x < MAX_X))						//範囲チェック
            {
            	if (wall_naname[x+1][y-1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    if(prev_dir != current_dir) { // 向きが変わった場合、回転の重みから直線の重みにリセット
                        weight = fast_naname_cost;
                    }
            		if ((map_naname[x][y] + weight) <= map_naname[x+1][y-1])				//まだ値が入っていなければ
            		{
            			map_naname[x+1][y-1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x+1, y-1, south_east, south_east, weight);
                    }
            	}
            }

            if(y > 0)	// 右45度回転
            {
            	if (wall_naname[x][y-1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_45OUT_END_TIME; // 向きを45度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x][y-1])				//まだ値が入っていなければ
            		{
            			map_naname[x][y-1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x, y-1, south_east, south, weight);
                    }
            	}
            }

            if(x < MAX_X)	// 左45度回転
            {
            	if (wall_naname[x+1][y] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_45OUT_END_TIME; // 向きを45度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y])				//まだ値が入っていなければ
            		{
            			map_naname[x+1][y] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x+1, y, south_east, east, weight);
                    }
            	}
            }

            if((x > 0) && (y > 0))	// 右90度回転
            {
            	if (wall_naname[x-1][y-1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_END_TIME; // 向きを90度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y-1])				//まだ値が入っていなければ
            		{
            			map_naname[x-1][y-1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x-1, y-1, south_east, south_west, weight);
                    }
            	}
            }

            if((x < MAX_X) && (y < MAX_Y))	// 左90度回転
            {
            	if (wall_naname[x+1][y+1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_END_TIME; // 向きを90度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y+1])				//まだ値が入っていなければ
            		{
            			map_naname[x+1][y+1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x+1, y+1, south_east, north_east, weight);
                    }
            	}
            }
            break;

        case south_west:
            if((y > 0) && (x > 0))						//範囲チェック
            {
            	if (wall_naname[x-1][y-1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    if(prev_dir != current_dir) { // 向きが変わった場合、回転の重みから直線の重みにリセット
                        weight = fast_naname_cost;
                    }
            		if ((map_naname[x][y] + weight) <= map_naname[x-1][y-1])				//まだ値が入っていなければ
            		{
            			map_naname[x-1][y-1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x-1, y-1, south_west, south_west, weight);
                    }
            	}
            }

            if(x > 0)	// 右45度回転
            {
            	if (wall_naname[x-1][y] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_45OUT_END_TIME; // 向きを45度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y])				//まだ値が入っていなければ
            		{
            			map_naname[x-1][y] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x-1, y, south_west, west, weight);
                    }
            	}
            }

            if(y > 0)	// 左45度回転
            {
            	if (wall_naname[x][y-1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_45OUT_END_TIME; // 向きを45度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x][y-1])				//まだ値が入っていなければ
            		{
            			map_naname[x][y-1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x, y-1, south_west, south, weight);
                    }
            	}
            }

            if((x > 0) && (y < MAX_Y))	// 右90度回転
            {
            	if (wall_naname[x-1][y+1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_END_TIME; // 向きを90度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y+1])				//まだ値が入っていなければ
            		{
            			map_naname[x-1][y+1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x-1, y+1, south_west, north_west, weight);
                    }
            	}
            }

            if((x < MAX_X) && (y > 0))	// 左90度回転
            {
            	if (wall_naname[x+1][y-1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_END_TIME; // 向きを90度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y-1])				//まだ値が入っていなければ
            		{
            			map_naname[x+1][y-1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x+1, y-1, south_west, south_east, weight);
                    }
            	}
            }
            break;

        case east:
            if(x < MAX_X) 						//範囲チェック
            {
            	if (wall_naname[x+1][y] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    if(prev_dir != current_dir) { // 向きが変わった場合、回転の重みから直線の重みにリセット
                        weight = fast_straight_cost;
                    }
            		if ((map_naname[x][y] + weight) <= map_naname[x+1][y])				//まだ値が入っていなければ
            		{
            			map_naname[x+1][y] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x+1, y, east, east, weight);
                    }
            	}
            }

            if((y > 0) && (x < MAX_X))	// 右45度回転
            {
            	if (wall_naname[x+1][y-1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_45IN_END_TIME; // 向きを45度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y-1])				//まだ値が入っていなければ
            		{
            			map_naname[x+1][y-1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x+1, y-1, east, south_east, weight);
                    }
            	}
            }

            if((y < MAX_Y) && (x < MAX_X))	// 左45度回転
            {
            	if (wall_naname[x+1][y+1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_45IN_END_TIME; // 向きを45度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x+1][y+1])				//まだ値が入っていなければ
            		{
            			map_naname[x+1][y+1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x+1, y+1, east, north_east, weight);
                    }
            	}
            }

            if(y > 0)	// 右90度回転
            {
            	if (wall_naname[x][y-1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_END_TIME; // 向きを90度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x][y-1])				//まだ値が入っていなければ
            		{
            			map_naname[x][y-1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x, y-1, east, south, weight);
                    }
            	}
            }

            if(y < MAX_Y)	// 左90度回転
            {
            	if (wall_naname[x][y+1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_END_TIME; // 向きを90度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x][y+1])				//まだ値が入っていなければ
            		{
            			map_naname[x][y+1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x, y+1, east, north, weight);
                    }
            	}
            }
            break;

        case west:
            if(x > 0) 						//範囲チェック
            {
            	if (wall_naname[x-1][y] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    if(prev_dir != current_dir) { // 向きが変わった場合、回転の重みから直線の重みにリセット
                        weight = fast_straight_cost;
                    }
            		if ((map_naname[x][y] + weight) <= map_naname[x-1][y])				//まだ値が入っていなければ
            		{
            			map_naname[x-1][y] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x-1, y, west, west, weight);
                    }
            	}
            }

            if((y < MAX_Y) && (x > 0))	// 右45度回転
            {
            	if (wall_naname[x-1][y+1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_45IN_END_TIME; // 向きを45度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y+1])				//まだ値が入っていなければ
            		{
            			map_naname[x-1][y+1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x-1, y+1, west, north_west, weight);
                    }
            	}
            }

            if((y > 0) && (x > 0))	// 左45度回転
            {
            	if (wall_naname[x-1][y-1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_45IN_END_TIME; // 向きを45度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x-1][y-1])				//まだ値が入っていなければ
            		{
            			map_naname[x-1][y-1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x-1, y-1, west, south_west, weight);
                    }
            	}
            }

            if(y < MAX_Y)	// 右90度回転
            {
            	if (wall_naname[x][y+1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_END_TIME; // 向きを90度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x][y+1])				//まだ値が入っていなければ
            		{
            			map_naname[x][y+1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x, y+1, west, north, weight);
                    }
            	}
            }

            if(y > 0)	// 左90度回転
            {
            	if (wall_naname[x][y-1] == NOWALL)//壁がなければ(maskの意味はstatic_parametersを参照)
            	{
                    weight = DAIKEI_END_TIME; // 向きを90度回転したため、45度の重みに設定
            		if((map_naname[x][y] + weight) <= map_naname[x][y-1])				//まだ値が入っていなければ
            		{
            			map_naname[x][y-1] = map_naname[x][y] + weight;	//値を代入
                        prioritize_straight_cost_recursion(x, y-1, west, south, weight);
                    }
            	}
            }
            break;

    }
}



void make_map_naname_recursion(int x, int y)	//歩数マップを作成する
{

	init_map_naname(x,y);											//Mapを初期化する
    
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
    for(int j = 0; j <MAZESIZE_Y; j++)						//迷路の大きさ分ループ(x座標)
    {
    	for(int i = 0; i < MAZESIZE_X; i++)					//迷路の大きさ分ループ(y座標)
    	{
                score = map_naname[i*2+1][j*2+1];

                sprintf(str, "%u", score);
                API_setText(i, j, str);
        }
    }
    for(int j = MAX_Y; j >= 0; j--)						//迷路の大きさ分ループ(x座標)
    {
    	for(int i = 0; i < MAX_X+1; i++)					//迷路の大きさ分ループ(y座標)
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



t_bool is_unknown(int x, int y)	//指定された区画が未探索か否かを判断する関数 未探索:true　探索済:false
{
	//座標x,yが未探索区間か否かを調べる
	
	if((wall[x][y].north == UNKNOWN) || (wall[x][y].east == UNKNOWN) || (wall[x][y].south == UNKNOWN) || (wall[x][y].west == UNKNOWN))//どこかの壁情報が不明のままであれば
	{			
		return true;	//未探索
	}
	else
	{
		return false;	//探索済
	}
}



int get_priority(int x, int y, t_direction dir)	//そのマスの情報から、優先度を算出する
{
	//座標x,yと、向いている方角dirから優先度を算出する
	
	//未探索が一番優先度が高い.(4)
	//それに加え、自分の向きと、行きたい方向から、
	//前(2)横(1)後(0)の優先度を付加する。

	int priority;								//優先度を記録する変数
	
	priority = 0;
	
	if(mypos.dir == dir)						//行きたい方向が現在の進行方向と同じ場合
	{
		priority = 2;
	}
	else if( ((4+mypos.dir-dir)%4) == 2)		//行きたい方向が現在の進行方向と逆の場合
	{
		priority = 0;
	}
	else										//それ以外(左右どちらか)の場合
	{
		priority = 1;
	}
	
	
	if(is_unknown(x,y) == true)
	{
		priority += 4;							//未探索の場合優先度をさらに付加
	}
	
	return priority;							//優先度を返す
	
}



int get_nextdir_naname(int x, int y, t_direction *dir)	
{
	//ゴール座標x,yに向かう場合、今どちらに行くべきかを判断する。
	int little,priority,tmp_priority;								//最小の値を探すために使用する変数
 
	little = 59999;													//最小歩数を255歩(mapがunsigned char型なので)に設定	

	priority = 0;													//優先度の初期値は0
	
	if( wall_naname[mypos.x*2+1][mypos.y*2+2] == NOWALL)			//北に壁がなければ
	{
		tmp_priority = get_priority(mypos.x, mypos.y + 1, north);	//優先度を算出
		if(map_naname[mypos.x*2+1][(mypos.y+1)*2+1] < little)						//一番歩数が小さい方向を見つける
		{
			little = map_naname[mypos.x*2+1][(mypos.y+1)*2+1];						//ひとまず北が歩数が小さい事にする
			*dir = north;											//方向を保存
			priority = tmp_priority;								//優先度を保存
		}
		else if(map_naname[mypos.x*2+1][(mypos.y+1)*2+1] == little)					//歩数が同じ場合は優先度から判断する
		{
			if(priority < tmp_priority )							//優先度を評価
			{
				*dir = north;										//方向を更新
				priority = tmp_priority;							//優先度を保存
			}
		}
	}
	
	if( wall_naname[mypos.x*2+2][mypos.y*2+1] == NOWALL)				//東に壁がなければ
	{
		tmp_priority = get_priority(mypos.x + 1, mypos.y, east);	//優先度を算出
		if(map_naname[(mypos.x + 1)*2+1][mypos.y*2+1] < little)						//一番歩数が小さい方向を見つける
		{
			little = map_naname[(mypos.x+1)*2+1][mypos.y*2+1];						//ひとまず東が歩数が小さい事にする
			*dir = east;											//方向を保存
			priority = tmp_priority;								//優先度を保存
		}
		else if(map_naname[(mypos.x + 1)*2+1][mypos.y*2+1] == little)				//歩数が同じ場合、優先度から判断
		{
			if(priority < tmp_priority)								//優先度を評価
			{
				*dir = east;										//方向を保存
				priority = tmp_priority;							//優先度を保存
			}
		}
	}
	
	if( wall_naname[mypos.x*2+1][mypos.y*2] == NOWALL)			//南に壁がなければ
	{
		tmp_priority = get_priority(mypos.x, mypos.y - 1, south);	//優先度を算出
		if(map_naname[mypos.x*2+1][(mypos.y - 1)*2+1] < little)						//一番歩数が小さい方向を見つける
		{
			little = map_naname[mypos.x*2+1][(mypos.y-1)*2+1];						//ひとまず南が歩数が小さい事にする
			*dir = south;											//方向を保存
			priority = tmp_priority;								//優先度を保存
		}
		else if(map_naname[mypos.x*2+1][(mypos.y - 1)*2+1] == little)				//歩数が同じ場合、優先度で評価
		{
			if(priority < tmp_priority)								//優先度を評価
			{
				*dir = south;										//方向を保存
				priority = tmp_priority;							//優先度を保存
			}
		}
	}
	
	if( wall_naname[mypos.x*2][mypos.y*2+1] == NOWALL)				//西に壁がなければ
	{
		tmp_priority = get_priority(mypos.x - 1, mypos.y, west);	//優先度を算出
		if(map_naname[(mypos.x-1)*2+1][mypos.y*2+1] < little)						//一番歩数が小さい方向を見つける
		{
			little = map_naname[(mypos.x-1)*2+1][mypos.y*2+1];						//西が歩数が小さい
			*dir = west;											//方向を保存
			priority = tmp_priority;								//優先度を保存
		}
		else if(map_naname[(mypos.x - 1)*2+1][mypos.y*2+1] == little)				//歩数が同じ場合、優先度で評価
		{
			*dir = west;											//方向を保存
			priority = tmp_priority;								//優先度を保存
		}
	}

    if(little == 59999) {
	unable_to_find_path_to_goal = 1;
	return stop;
    }

	return ( (int)( ( 4 + *dir - mypos.dir) % 4 ) );				//どっちに向かうべきかを返す。
}



int get_nextdir(int x, int y, int mask, t_direction *dir)	
{
	//ゴール座標x,yに向かう場合、今どちらに行くべきかを判断する。
	//探索、最短の切り替えのためのmaskを指定、dirは方角を示す
	int little,priority,tmp_priority;								//最小の値を探すために使用する変数
 
	make_map(x,y,mask);												//歩数Map生成
	little = 255;													//最小歩数を255歩(mapがunsigned char型なので)に設定	

	priority = 0;													//優先度の初期値は0
	
		//maskの意味はstatic_parameter.hを参照
	if( (wall[mypos.x][mypos.y].north & mask) == NOWALL)			//北に壁がなければ
	{
		tmp_priority = get_priority(mypos.x, mypos.y + 1, north);	//優先度を算出
		if(map[mypos.x][mypos.y+1] < little)						//一番歩数が小さい方向を見つける
		{
			little = map[mypos.x][mypos.y+1];						//ひとまず北が歩数が小さい事にする
			*dir = north;											//方向を保存
			priority = tmp_priority;								//優先度を保存
		}
		else if(map[mypos.x][mypos.y+1] == little)					//歩数が同じ場合は優先度から判断する
		{
			if(priority < tmp_priority )							//優先度を評価
			{
				*dir = north;										//方向を更新
				priority = tmp_priority;							//優先度を保存
			}
		}
	}
	
	if( (wall[mypos.x][mypos.y].east & mask) == NOWALL)				//東に壁がなければ
	{
		tmp_priority = get_priority(mypos.x + 1, mypos.y, east);	//優先度を算出
		if(map[mypos.x + 1][mypos.y] < little)						//一番歩数が小さい方向を見つける
		{
			little = map[mypos.x+1][mypos.y];						//ひとまず東が歩数が小さい事にする
			*dir = east;											//方向を保存
			priority = tmp_priority;								//優先度を保存
		}
		else if(map[mypos.x + 1][mypos.y] == little)				//歩数が同じ場合、優先度から判断
		{
			if(priority < tmp_priority)								//優先度を評価
			{
				*dir = east;										//方向を保存
				priority = tmp_priority;							//優先度を保存
			}
		}
	}
	
	if( (wall[mypos.x][mypos.y].south & mask) == NOWALL)			//南に壁がなければ
	{
		tmp_priority = get_priority(mypos.x, mypos.y - 1, south);	//優先度を算出
		if(map[mypos.x][mypos.y - 1] < little)						//一番歩数が小さい方向を見つける
		{
			little = map[mypos.x][mypos.y-1];						//ひとまず南が歩数が小さい事にする
			*dir = south;											//方向を保存
			priority = tmp_priority;								//優先度を保存
		}
		else if(map[mypos.x][mypos.y - 1] == little)				//歩数が同じ場合、優先度で評価
		{
			if(priority < tmp_priority)								//優先度を評価
			{
				*dir = south;										//方向を保存
				priority = tmp_priority;							//優先度を保存
			}
		}
	}
	
	if( (wall[mypos.x][mypos.y].west & mask) == NOWALL)				//西に壁がなければ
	{
		tmp_priority = get_priority(mypos.x - 1, mypos.y, west);	//優先度を算出
		if(map[mypos.x-1][mypos.y] < little)						//一番歩数が小さい方向を見つける
		{
			little = map[mypos.x-1][mypos.y];						//西が歩数が小さい
			*dir = west;											//方向を保存
			priority = tmp_priority;								//優先度を保存
		}
		else if(map[mypos.x - 1][mypos.y] == little)				//歩数が同じ場合、優先度で評価
		{
			*dir = west;											//方向を保存
			priority = tmp_priority;								//優先度を保存
		}
	}

    if(little == 255) {
	unable_to_find_path_to_goal = 1;
	return stop;
    }

	return ( (int)( ( 4 + *dir - mypos.dir) % 4 ) );				//どっちに向かうべきかを返す。
}



int fast_full_search_adachi_slalom(int startX, int endX, int startY, int endY)
{
//引数gx,gyに向かって足立法で迷路を探索する
    short tmp_x, tmp_y;
    short gx, gy;
    short no_goal_exit_loop_flag = 0;
	t_direction glob_nextdir;									//次に向かう方向を記録する変数

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

	            switch(get_nextdir(gx,gy,MASK_SEARCH,&glob_nextdir))		//次に行く方向を戻り値とする関数を呼ぶ
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
			    
	                    //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);	//一区画進む
                        API_moveForward();
	                    break;
	                
	                case right:
                        fprintf(stderr, "right\n");
                        fflush(stderr);
	                    //rotate(right,1);									//右に曲がって
	                    //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);	//半区画進む
                        API_turnRight();
                        API_moveForward();
	                    break;
	                
	                case left:
                        fprintf(stderr, "left\n");
                        fflush(stderr);
	                    //rotate(left,1);										//左に曲がって
	                    //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);	//半区画進む
                        API_turnLeft();
                        API_moveForward();
	                    break;
	                
	                case rear:
                        fprintf(stderr, "rear\n");
                        fflush(stderr);
	                    //rotate(left,2);										//180ターン
	                    //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);	//半区画進む
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
	            mypos.dir = glob_nextdir;									//方向を更新


	            //向いた方向によって自分の座標を更新する
	            switch(mypos.dir)
	            {
	                case north:
                        fprintf(stderr, "current direction: north \n");
                        fflush(stderr);
	                    mypos.y++;	//北を向いた時はY座標を増やす
	                    break;
	                    
	                case east:
                        fprintf(stderr, "current direction: east\n");
	                    mypos.x++;	//東を向いた時はX座標を増やす
	                    break;
	                    
	                case south:
                        fprintf(stderr, "current direction: south\n");
	                    mypos.y--;	//南を向いた時はY座標を減らす
	                    break;
	                
	                case west:
                        fprintf(stderr, "current direction: west\n");
	                    mypos.x--;	//西を向いたときはX座標を減らす
	                    break;

	            }
	            
	            while(((mypos.x != gx) || (mypos.y != gy)) && (!no_goal_exit_loop_flag)){							//ゴールするまで繰り返す
                    fprintf(stderr, "current x: %d, y: %d | goal:%d, %d \n", mypos.x, mypos.y, gx, gy);
                    fflush(stderr);
                    API_setColor(mypos.x, mypos.y, 'A');
		    	    if (already_passed_boolean_map[mypos.y*16 + mypos.x] == 0){
		                    already_passed_boolean_map[mypos.y*16 + mypos.x] = 1;
		                    set_wall(mypos.x,mypos.y);					//壁をセット
	                }//*/
			   
			    if(is_unknown(mypos.x,mypos.y) == true)
		            {
				set_wall(mypos.x,mypos.y);                    //壁をセット
			    }
			       

	                switch(get_nextdir(gx,gy,MASK_SEARCH,&glob_nextdir))			//次に行く方向を戻り値とする関数を呼ぶ
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
	                        //if (already_passed_boolean_map[tmp_y*16 + tmp_x] == 1)	{//既知区画をスキップ
	                        //    search_straight_count+=1;
	                        //    mypos.x=tmp_x;
	                        //    mypos.y=tmp_y;
	                        //    mypos.dir = glob_nextdir;
	                        //    continue;
	                        //} else {//未知区画
                        if (search_straight_count>0){
                            for(short z=0; z<(search_straight_count+1); z++){
                                API_moveForward();
                            }
                            API_moveForward();
					    //straight_for_search(SECTION*(search_straight_count+1),700.0,SEARCH_SPEED);
                        } else {
                            API_moveForward();
					    //straight_for_search(SECTION,SEARCH_SPEED,SEARCH_SPEED);	//一区画進む
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
	                         
	                       		//半区画進んで
	                        //rotate(left,2);					//180ターン
	                        //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);		//半区画進む
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

	            
	                mypos.dir = glob_nextdir;										//方向を更新
	                
	                //向いた方向によって自分の座標を更新する
	                switch(mypos.dir)
	                {
	                    case north:
                            fprintf(stderr, "current dir: north\n");
                            fflush(stderr);
	                        mypos.y++;	//北を向いた時はY座標を増やす
	                        break;
	                        
	                    case east:
                            fprintf(stderr, "current dir: east\n");
	                        mypos.x++;	//東を向いた時はX座標を増やす
	                        break;
	                        
	                    case south:
                            fprintf(stderr, "current dir: south\n");
	                        mypos.y--;	//南を向いた時はY座標を減らす
	                        break;
	                    
	                    case west:
                            fprintf(stderr, "current dir: west\n");
	                        mypos.x--;	//西を向いたときはX座標を減らす
	                        break;

	                }
	                
	            }
		    if (((mypos.x == gx) && (mypos.y == gy)) && (already_passed_boolean_map[mypos.y*16 + mypos.x] == 0)){
                    fprintf(stderr, "current x: %d, y: %d | goal:%d, %d \n", mypos.x, mypos.y, gx, gy);
                    fprintf(stderr, "GOAL!!!!\n");
                    set_wall(mypos.x,mypos.y);                    //壁をセット
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
			 	set_wall(mypos.x,mypos.y);                    //壁をセット
		   	}
	                //straight_for_search(HALF_SECTION,SEARCH_SPEED,0);	//ゴール.半区画進む
	            }
		  
	    }
        }
    }
    //*/
    return 0;
}



//スラローム走行の最短経路マップを作成
void create_fast_run_slalom_map(int x, int y)
{
	t_direction glob_nextdir;
	float straight_count = -0.5;
    int action_id = 0;
    shortest_route_action[action_id] = M_FRONT;
    shortest_route_action_times[action_id] = straight_count;

	//現在の向きから、次に行くべき方向へ向く
	switch(get_nextdir(x,y,MASK_SECOND,&glob_nextdir))	//次に行く方向を戻り値とする関数を呼ぶ
	{
		case front:
			straight_count++;							//前向きだった場合は直線を走る距離を伸ばす
			break;
		
		case right:										//右に向く
			straight_count++;							//前向きだった場合は直線を走る距離を伸ばす
			break;
		
		case left:										//左に向く
			straight_count++;							//前向きだった場合は直線を走る距離を伸ばす
			break;
		
		case rear:										//後ろに向く
			straight_count++;							//前向きだった場合は直線を走る距離を伸ばす
			break;
	}

    shortest_route_action[action_id] = M_FRONT;
    shortest_route_action_times[action_id] = straight_count;
	mypos.dir = glob_nextdir;							//自分の向きを更新


	//向いた方向によって自分の座標を更新する
	switch(mypos.dir)
	{
		case north:
			mypos.y++;	//北を向いた時はY座標を増やす
			break;
			
		case east:
			mypos.x++;	//東を向いた時はX座標を増やす
			break;
			
		case south:
			mypos.y--;	//南を向いた時はY座標を減らす
			break;
		
		case west:
			mypos.x--;	//西を向いたときはX座標を減らす
			break;
	}

	while((mypos.x != x) || (mypos.y != y)){	//ゴールするまで繰り返す
		switch(get_nextdir(x,y,MASK_SECOND,&glob_nextdir))	//次に行く方向を戻り値とする関数を呼ぶ
		{
			case front:	//直線をまとめて走るようにする
				straight_count += 1.0;
				break;
			
			case right:
		                if (straight_count > 0.0) {
		                    //前進を登録
		                    shortest_route_action[action_id] = M_FRONT;
		                    shortest_route_action_times[action_id] = straight_count;
		                    action_id++;

		                    //回転を登録
		                    shortest_route_action[action_id] = M_RIGHT;
		                    shortest_route_action_times[action_id] = 1;
		                    action_id++;
		                    straight_count = 0.0;
		                }
		                else {
		                    //回転を登録
		                    shortest_route_action[action_id] = M_RIGHT;
		                    shortest_route_action_times[action_id] = 1;
		                    action_id++;
		                }
				break;
			
			case left:
		                if (straight_count > 0.0) {
		                    //前進を登録
		                    shortest_route_action[action_id] = M_FRONT;
		                    shortest_route_action_times[action_id] = straight_count;
		                    action_id++;

		                    //回転を登録
		                    shortest_route_action[action_id] = M_LEFT;
		                    shortest_route_action_times[action_id] = 1;
		                    action_id++;
		                    straight_count = 0.0;
		                }
		                else {
		                    //回転を登録
		                    shortest_route_action[action_id] = M_LEFT;
		                    shortest_route_action_times[action_id] = 1;
		                    action_id++;
		                }
		}
	
		mypos.dir = glob_nextdir;							//自分の向きを修正
		
		//向いた方向によって自分の座標を更新する
		switch(mypos.dir)
		{
			case north:
				mypos.y++;	//北を向いた時はY座標を増やす
				break;
				
			case east:
				mypos.x++;	//東を向いた時はX座標を増やす
				break;
				
			case south:
				mypos.y--;	//南を向いた時はY座標を減らす
				break;
			
			case west:
				mypos.x--;	//西を向いたときはX座標を減らす
				break;

		}
	}
    shortest_route_action[action_id] = M_FRONT;
    shortest_route_action_times[action_id] = straight_count + 0.5;
    action_id++;
    shortest_route_action[action_id] = M_REAR;
}


//最速斜め経路用のスラローム走行の最短経路マップを作成
void create_fast_run_diagonal_map(int x, int y)
{
	t_direction glob_nextdir;
	float straight_count = -0.5;
    int action_id = 0;
    shortest_route_action[action_id] = M_FRONT;
    shortest_route_action_times[action_id] = straight_count;

    API_setColor(mypos.x, mypos.y, color);
	//現在の向きから、次に行くべき方向へ向く
	switch(get_nextdir_naname(x,y,&glob_nextdir))	//次に行く方向を戻り値とする関数を呼ぶ
	{
		case front:
			straight_count++;							//前向きだった場合は直線を走る距離を伸ばす
			break;
		
		case right:										//右に向く
			straight_count++;							//前向きだった場合は直線を走る距離を伸ばす
			break;
		
		case left:										//左に向く
			straight_count++;							//前向きだった場合は直線を走る距離を伸ばす
			break;
		
		case rear:										//後ろに向く
			straight_count++;							//前向きだった場合は直線を走る距離を伸ばす
			break;
	}

    shortest_route_action[action_id] = M_FRONT;
    shortest_route_action_times[action_id] = straight_count;
	mypos.dir = glob_nextdir;							//自分の向きを更新


	//向いた方向によって自分の座標を更新する
	switch(mypos.dir)
	{
		case north:
			mypos.y++;	//北を向いた時はY座標を増やす
			break;
			
		case east:
			mypos.x++;	//東を向いた時はX座標を増やす
			break;
			
		case south:
			mypos.y--;	//南を向いた時はY座標を減らす
			break;
		
		case west:
			mypos.x--;	//西を向いたときはX座標を減らす
			break;
	}

	while((mypos.x != x) || (mypos.y != y)){	//ゴールするまで繰り返す
        API_setColor(mypos.x, mypos.y, color);
		switch(get_nextdir_naname(x,y,&glob_nextdir))	//次に行く方向を戻り値とする関数を呼ぶ
		{
			case front:	//直線をまとめて走るようにする
				straight_count += 1.0;
				break;
			
			case right:
		                if (straight_count > 0.0) {
		                    //前進を登録
		                    shortest_route_action[action_id] = M_FRONT;
		                    shortest_route_action_times[action_id] = straight_count;
		                    action_id++;

		                    //回転を登録
		                    shortest_route_action[action_id] = M_RIGHT;
		                    shortest_route_action_times[action_id] = 1;
		                    action_id++;
		                    straight_count = 0.0;
		                }
		                else {
		                    //回転を登録
		                    shortest_route_action[action_id] = M_RIGHT;
		                    shortest_route_action_times[action_id] = 1;
		                    action_id++;
		                }
				break;
			
			case left:
		                if (straight_count > 0.0) {
		                    //前進を登録
		                    shortest_route_action[action_id] = M_FRONT;
		                    shortest_route_action_times[action_id] = straight_count;
		                    action_id++;

		                    //回転を登録
		                    shortest_route_action[action_id] = M_LEFT;
		                    shortest_route_action_times[action_id] = 1;
		                    action_id++;
		                    straight_count = 0.0;
		                }
		                else {
		                    //回転を登録
		                    shortest_route_action[action_id] = M_LEFT;
		                    shortest_route_action_times[action_id] = 1;
		                    action_id++;
		                }
		}
	
		mypos.dir = glob_nextdir;							//自分の向きを修正
		
		//向いた方向によって自分の座標を更新する
		switch(mypos.dir)
		{
			case north:
				mypos.y++;	//北を向いた時はY座標を増やす
				break;
				
			case east:
				mypos.x++;	//東を向いた時はX座標を増やす
				break;
				
			case south:
				mypos.y--;	//南を向いた時はY座標を減らす
				break;
			
			case west:
				mypos.x--;	//西を向いたときはX座標を減らす
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
        switch(shortest_route_action[action_id])	//次に行く方向を戻り値とする関数を呼ぶ
    	{
	    case M_FRONT:
            fflush(stderr);
            //現在の姿勢が右斜め状態
            if (current_dir == M_RIGHT45) { //45度出口
                current_dir = M_FRONT; //姿勢を更新
                diagonal_route_action[diagonal_action_id] = M_LEFT45OUT;
                diagonal_route_action_times[diagonal_action_id] = 1.0;
                diagonal_action_id++;

                // 斜め45度出口後、姿勢を修正するための区画を確保
                shortest_route_action_times[action_id] -= 0.5;
            }

            //現在の姿勢が左斜め状態
            if (current_dir == M_LEFT45) { //45度出口
                current_dir = M_FRONT; //姿勢を更新
                diagonal_route_action[diagonal_action_id] = M_RIGHT45OUT;
                diagonal_route_action_times[diagonal_action_id] = 1.0;
                diagonal_action_id++;

                // 斜め45度出口後、姿勢を修正するための区画を確保
                shortest_route_action_times[action_id] -= 0.5;
            }

            //以下の現在姿勢は真っ直ぐの状態
            //最後の直線であるかどうかの確認
            if (shortest_route_action[action_id+1] == M_REAR) {
                diagonal_route_action[diagonal_action_id] = M_FRONT;
                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id];
                diagonal_action_id++;
                action_id++;

                diagonal_route_action[diagonal_action_id] = M_REAR;
                break;
            }

            //左45度斜め開始
            //進→左→右 
            if ((shortest_route_action[action_id+1] == M_LEFT) && (shortest_route_action[action_id+2] == M_RIGHT)) {
                diagonal_route_action[diagonal_action_id] = M_FRONT;
                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id] - 0.5; //45度斜め回転の準備区画
                diagonal_action_id++;

                current_dir = M_LEFT45;//姿勢を更新
                diagonal_route_action[diagonal_action_id] = M_LEFT45;
                diagonal_route_action_times[diagonal_action_id] = 1.0;
                diagonal_action_id++;

                diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                diagonal_route_action_times[diagonal_action_id] = 1.0;
                diagonal_action_id++;

                action_id += 3; //次の行動までスキップ
                break;
            } 
            
            //右45度斜め開始
            //進→右→左 
            if ((shortest_route_action[action_id+1] == M_RIGHT) && (shortest_route_action[action_id+2] == M_LEFT)) {
                diagonal_route_action[diagonal_action_id] = M_FRONT;
                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id] - 0.5; //45度斜め回転の準備区画
                diagonal_action_id++;

                current_dir = M_RIGHT45;//姿勢を更新
                diagonal_route_action[diagonal_action_id] = M_RIGHT45;
                diagonal_route_action_times[diagonal_action_id] = 1.0;
                diagonal_action_id++;

                diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                diagonal_route_action_times[diagonal_action_id] = 1.0;
                diagonal_action_id++;

                action_id += 3; //次の行動までスキップ
                break;
            }
            
            //左90度回転
            //進→左→進
            if ((shortest_route_action[action_id+1] == M_LEFT) && (shortest_route_action[action_id+2] == M_FRONT)) {	  
		//左90度回転大回り
                /*if (shortest_route_action_times[action_id] >= 2.0 && shortest_route_action_times[action_id+2] >= 2.5) {
	    		diagonal_route_action[diagonal_action_id] = M_FRONT;
	                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id] - 0.5;
	                diagonal_action_id++;

	                current_dir = M_FRONT;//姿勢を更新
	                diagonal_route_action[diagonal_action_id] = M_LEFT_F;
	                diagonal_route_action_times[diagonal_action_id] = 1.0;
	                diagonal_action_id++;

	                shortest_route_action_times[action_id+2] -= 1.5;
                } else {*/
	                diagonal_route_action[diagonal_action_id] = M_FRONT;
	                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id];
	                diagonal_action_id++;

	                current_dir = M_FRONT;//姿勢を更新
	                diagonal_route_action[diagonal_action_id] = M_LEFT;
	                diagonal_route_action_times[diagonal_action_id] = 1.0;
	                diagonal_action_id++;

	                shortest_route_action_times[action_id+2] -= 0.25;
		//}
                action_id += 2; //次の直線行動までスキップ
                break;
            }

            //右90度回転
            //進→右→進
            if ((shortest_route_action[action_id+1] == M_RIGHT) && (shortest_route_action[action_id+2] == M_FRONT)) {
                /*if (shortest_route_action_times[action_id] >= 2.0 && shortest_route_action_times[action_id+2] >= 2.5) {
	    		diagonal_route_action[diagonal_action_id] = M_FRONT;
	                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id] - 0.5;
	                diagonal_action_id++;

	                current_dir = M_FRONT;//姿勢を更新
	                diagonal_route_action[diagonal_action_id] = M_RIGHT_F;
	                diagonal_route_action_times[diagonal_action_id] = 1.0;
	                diagonal_action_id++;

	                shortest_route_action_times[action_id+2] -= 1.5;
                } else {*/
	                diagonal_route_action[diagonal_action_id] = M_FRONT;
	                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id];
	                diagonal_action_id++;

	                current_dir = M_FRONT; //姿勢を更新
	                diagonal_route_action[diagonal_action_id] = M_RIGHT;
	                diagonal_route_action_times[diagonal_action_id] = 1.0;
	                diagonal_action_id++;

	                shortest_route_action_times[action_id+2] -= 0.25;
		//}
                action_id += 2; //次の行動までスキップ
                break;
            }

            //左135度回転
            //進→左→左→右
            if ((shortest_route_action[action_id+1] == M_LEFT) && (shortest_route_action[action_id+2] == M_LEFT) && (shortest_route_action[action_id+3] == M_RIGHT)) {
                diagonal_route_action[diagonal_action_id] = M_FRONT;
                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id]-0.5;
                diagonal_action_id++;

                current_dir = M_LEFT45; //姿勢を更新、90+斜め45度であるため、姿勢は斜め45度と同じ
                diagonal_route_action[diagonal_action_id] = M_LEFT135;
                diagonal_route_action_times[diagonal_action_id] = 1.0;
                diagonal_action_id++;

                // 左135→右V90でない時
                if (shortest_route_action[action_id+4] != M_RIGHT) {
                    diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                    diagonal_route_action_times[diagonal_action_id] = 1.0;
                    diagonal_action_id++;
                }

                action_id += 4; //次の行動までスキップ
                break;
            }

            //右135度回転
            //進→右→右→左
            if ((shortest_route_action[action_id+1] == M_RIGHT) && (shortest_route_action[action_id+2] == M_RIGHT) && (shortest_route_action[action_id+3] == M_LEFT)) {
                diagonal_route_action[diagonal_action_id] = M_FRONT;
                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id]-0.5;
                diagonal_action_id++;

                current_dir = M_RIGHT45; //姿勢を更新、90+斜め45度であるため、姿勢は斜め45度と同じ
                diagonal_route_action[diagonal_action_id] = M_RIGHT135;
                diagonal_route_action_times[diagonal_action_id] = 1.0;
                diagonal_action_id++;

                // 右135→左V90でない時
                if (shortest_route_action[action_id+4] != M_LEFT) {
                    diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                    diagonal_route_action_times[diagonal_action_id] = 1.0;
                    diagonal_action_id++;
                }

                action_id += 4; //次の行動までスキップ
                break;
            }

            //左180度回転
            //進→左→左→進
            if ((shortest_route_action[action_id+1] == M_LEFT) && (shortest_route_action[action_id+2] == M_LEFT) && (shortest_route_action[action_id+3] == M_FRONT)) {
                /*if ((shortest_route_action_times[action_id+3] - 0.5) >= 1.0) {
			diagonal_route_action[diagonal_action_id] = M_FRONT;
	                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id]-0.2;
	                diagonal_action_id++;

	                current_dir = M_FRONT; //姿勢を更新
	                diagonal_route_action[diagonal_action_id] = M_LEFT180_F;
	                diagonal_route_action_times[diagonal_action_id] = 1.0;
	                diagonal_action_id++;
	                shortest_route_action_times[action_id+3] -= 0.01;
	                //shortest_route_action_times[action_id+3] -= 0.7;
		} else {*/
		        diagonal_route_action[diagonal_action_id] = M_FRONT;
	                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id]-0.1;
	                diagonal_action_id++;

	                current_dir = M_FRONT; //姿勢を更新
	                diagonal_route_action[diagonal_action_id] = M_LEFT180;
	                diagonal_route_action_times[diagonal_action_id] = 1.0;
	                diagonal_action_id++;
			shortest_route_action_times[action_id+3] -= (0.25 + 0.07);
	                //shortest_route_action_times[action_id+3] -= 0.5;
		//}
                action_id += 3; //次の行動までスキップ
                break;
            }

            //右180度回転
            //進→右→右→進
            if ((shortest_route_action[action_id+1] == M_RIGHT) && (shortest_route_action[action_id+2] == M_RIGHT) && (shortest_route_action[action_id+3] == M_FRONT)) {
		/*if ((shortest_route_action_times[action_id+3] - 0.5) >= 1.0) {
			diagonal_route_action[diagonal_action_id] = M_FRONT;
	                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id]-0.2;
	                diagonal_action_id++;

	                current_dir = M_FRONT; //姿勢を更新
	                diagonal_route_action[diagonal_action_id] = M_RIGHT180_F;
	                diagonal_route_action_times[diagonal_action_id] = 1.0;
	                diagonal_action_id++;
	                shortest_route_action_times[action_id+3] -= 0.01;
	                //shortest_route_action_times[action_id+3] -= 0.7;
		} else {*/
	                diagonal_route_action[diagonal_action_id] = M_FRONT;
	                diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id]-0.1;
	                diagonal_action_id++;

	                current_dir = M_FRONT; //姿勢を更新
	                diagonal_route_action[diagonal_action_id] = M_RIGHT180;
	                diagonal_route_action_times[diagonal_action_id] = 1.0;
	                diagonal_action_id++;
			shortest_route_action_times[action_id+3] -= (0.25 + 0.07);
	                //shortest_route_action_times[action_id+3] -= 0.5;
		//}
                action_id += 3; //次の行動までスキップ
                break;
            }

	        diagonal_route_action[diagonal_action_id] = M_FRONT;
	        diagonal_route_action_times[diagonal_action_id] = shortest_route_action_times[action_id];
	        diagonal_action_id++;
	        action_id++;
	        current_dir = M_FRONT;//姿勢を更新
		
        //##################################右回転######################################
		case M_RIGHT:
            if (current_dir == M_FRONT) {break;}

            if (current_dir == M_LEFT45) { //右回転直後に右回転する状態(left45==右回転直後) → 右+右状態
                //右→右→進 == 右V90度1回転 → 右45度(45度はfrontのケースで行う)
                if (shortest_route_action[action_id+1] == M_FRONT) {
                    diagonal_route_action[diagonal_action_id] = M_RIGHTV90;
                    diagonal_route_action_times[diagonal_action_id] = 1.0;
                    diagonal_action_id++;

                    action_id += 1; //次の行動までスキップ
                    break;
                }

                //右→右→左 == 右V90度回転+ 斜め前進
                if (shortest_route_action[action_id+1] == M_LEFT) {
                    current_dir = M_RIGHT45; //45度斜めと同じ姿勢に修正
                    diagonal_route_action[diagonal_action_id] = M_RIGHTV90;
                    diagonal_route_action_times[diagonal_action_id] = 1.0;
                    diagonal_action_id++;

                    diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                    diagonal_route_action_times[diagonal_action_id] = 1.0;
                    diagonal_action_id++;

                    action_id += 2; //次の行動までスキップ
                    break;
                }
            }

            if (current_dir == M_RIGHT45) {//左回転直後に右回転する状態(left45==右回転直後) → 左+右状態
                //左→右→進 == 斜め前進 → 右45度(45度はfrontのケースで行う)
                if ((shortest_route_action[action_id+1] == M_FRONT)) {
                    current_dir = M_LEFT45; //次のステップで姿勢を真っ直ぐに直すために、45度斜めと同じ姿勢に修正
                    if (diagonal_route_action[diagonal_action_id-1] == M_FRONT_DIAGONAL) {
                        diagonal_route_action_times[diagonal_action_id-1] += 1.0;
                    } else {
                        diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                        diagonal_route_action_times[diagonal_action_id] = 1.0;
                        diagonal_action_id++;
                    }

                    action_id += 1; //次の行動までスキップ
                    break;
                }

                //左→右→左 == 斜め前進x2 
                if (shortest_route_action[action_id+1] == M_LEFT) {
                    if (diagonal_route_action[diagonal_action_id-1] == M_FRONT_DIAGONAL) {
                        diagonal_route_action_times[diagonal_action_id-1] += 2.0;
                    } else {
                        diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                        diagonal_route_action_times[diagonal_action_id] = 2.0;
                        diagonal_action_id++;
                    }

                    action_id += 2; //次の行動までスキップ
                    break;
                }

                //左→右→右 == 斜め前進 + 右V90
                if (shortest_route_action[action_id+1] == M_RIGHT) {
                    current_dir = M_LEFT45; //次のステップで姿勢を真っ直ぐに直すために、45度斜めと同じ姿勢に修正
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

                    action_id += 2; //次の行動までスキップ
                    break;
                }
            }
		
        //##################################左回転######################################
		case M_LEFT:
            //姿勢斜め状態で左回転
            if (current_dir == M_FRONT) {break;}

            if (current_dir == M_LEFT45) { //右回転直後に左回転する状態(left45==右回転直後) → 右+左状態
                //右→左→進 == 斜め前進 + 左45度
                if (shortest_route_action[action_id+1] == M_FRONT) {
                    current_dir = M_RIGHT45; //45度斜めと同じ姿勢に修正
                    if (diagonal_route_action[diagonal_action_id-1] == M_FRONT_DIAGONAL) {
                        diagonal_route_action_times[diagonal_action_id-1] += 1.0;
                    } else {
                        diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                        diagonal_route_action_times[diagonal_action_id] = 1.0;
                        diagonal_action_id++;
                    }

                    action_id += 1; //次の行動までスキップ
                    break;
                }

                //右→左→左 == 斜め前進 + 左V90度
                if (shortest_route_action[action_id+1] == M_LEFT) {
                    current_dir = M_RIGHT45; //次のステップで姿勢を真っ直ぐに直すために、45度斜めと同じ姿勢に修正
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

                    action_id += 2; //次の行動までスキップ
                    break;
                }

                //右→左→右 == 斜め前進x2
                if (shortest_route_action[action_id+1] == M_RIGHT) {
                    if (diagonal_route_action[diagonal_action_id-1] == M_FRONT_DIAGONAL) {
                        diagonal_route_action_times[diagonal_action_id-1] += 2.0;
                    } else {
                        diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                        diagonal_route_action_times[diagonal_action_id] = 2.0;
                        diagonal_action_id++;
                    }

                    action_id += 2; //次の行動までスキップ
                    break;
                }
            }

            if (current_dir == M_RIGHT45) {//左回転直後に左回転する状態(left45==右回転直後) → 左+左状態
                //左→左→進 == 左V90 + 右45度
                if (shortest_route_action[action_id+1] == M_FRONT) {
                    diagonal_route_action[diagonal_action_id] = M_LEFTV90;
                    diagonal_route_action_times[diagonal_action_id] = 1.0;
                    diagonal_action_id++;

                    action_id += 1; //次の行動までスキップ
                    break;
                }

                //左→左→右 == 左V90 + 斜め前進
                if (shortest_route_action[action_id+1] == M_RIGHT) {
                    current_dir = M_LEFT45; //45度斜めと同じ姿勢に修正
                    diagonal_route_action[diagonal_action_id] = M_LEFTV90;
                    diagonal_route_action_times[diagonal_action_id] = 1.0;
                    diagonal_action_id++;

                    diagonal_route_action[diagonal_action_id] = M_FRONT_DIAGONAL;
                    diagonal_route_action_times[diagonal_action_id] = 1;
                    diagonal_action_id++;

                    action_id += 2; //次の行動までスキップ
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
    //引数の座標x,yに向かって最短走行する
    unsigned int total_runtime = 0;
	int action_id = 1;

	if (shortest_route_action_times[0] > 1.0) {
		total_runtime += (unsigned int) (((shortest_route_action_times[0]*SECTION+30.0)/ZENSHIN_MAX_SPEED) *1000); // cost milisecond runtime
	} else {	//1区画は直線を走る
		total_runtime += (unsigned int) (((shortest_route_action_times[0]*SECTION+30.0)/DAIKEI_ZENSHIN) *1000); // cost milisecond runtime
	}

	while(shortest_route_action[action_id] != M_REAR){	//ゴールするまで繰り返す
		switch(shortest_route_action[action_id])	//次に行く方向を戻り値とする関数を呼ぶ
		{
			case M_FRONT:	//直線をまとめて走るようにする
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

    //引数の座標x,yに向かって最短走行する
    fprintf(stderr, "start calculate diagonal_route time\n");
    fflush(stderr);
    
    int action_id = 1;
    float prev_spd, next_spd;
    float diagonal_len, len;
    float max_spd;
    unsigned int total_runtime = 0;
    
    if (diagonal_route_action_times[0] > 1.0) {	 //長い直線を全速で走る
		total_runtime += (unsigned int) (((diagonal_route_action_times[0]*SECTION+30.0)/ZENSHIN_MAX_SPEED) *1000); // cost milisecond runtime
    } else {	//1区画は直線を走る
		total_runtime += (unsigned int) (((diagonal_route_action_times[0]*SECTION+30.0)/DAIKEI_ZENSHIN) *1000); // cost milisecond runtime
    }

    while(diagonal_route_action[action_id] != M_REAR){	//ゴールするまで繰り返す
        switch(diagonal_route_action[action_id]) {	//次に行く方向を戻り値とする関数を呼ぶ 
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
		
            case M_RIGHT45:		//右45
                total_runtime += DAIKEI_45IN_END_TIME; // cost milisecond runtime
                break;

            case M_LEFT45:		//左45
                total_runtime += DAIKEI_45IN_END_TIME; // cost milisecond runtime
                break;

            case M_RIGHTV90:		//右V90
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

            case M_LEFTV90:		//左V90
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

            case M_RIGHT135:		//右135
                total_runtime += DAIKEI_135IN_END_TIME; // cost milisecond runtime
                break;

            case M_LEFT135:		//左135
                total_runtime += DAIKEI_135IN_END_TIME; // cost milisecond runtime
                break;

            case M_RIGHT180:	    //右180
                total_runtime += DAIKEI_OOMAWARI180_END_TIME; // cost milisecond runtime
                total_runtime += (unsigned int) (((HALF_SECTION/2)/DAIKEI_ZENSHIN) *1000); // cost milisecond runtime
                break;

            case M_LEFT180:     	//左180
                total_runtime += DAIKEI_OOMAWARI180_END_TIME; // cost milisecond runtime
                total_runtime += (unsigned int) (((HALF_SECTION/2)/DAIKEI_ZENSHIN) *1000); // cost milisecond runtime
                break;

            case M_RIGHT45OUT:	 //右45出口
                total_runtime += DAIKEI_45OUT_END_TIME; // cost milisecond runtime
                break;

            case M_LEFT45OUT:	     //左45出口
                total_runtime += DAIKEI_45OUT_END_TIME; // cost milisecond runtime
                break;

            case M_FRONT_DIAGONAL: //斜め前進
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
	//ゴール座標x,yに向かう場合、今どちらに行くべきかを判断する。
	//探索、最短の切り替えのためのmaskを指定、dirは方角を示す
	int little,priority,tmp_priority;								//最小の値を探すために使用する変数
 
	make_map(x,y,mask);												//歩数Map生成
	//make_fast_run_map_with_run_time(x,y,mask);												//歩数Map生成
	little = 255;													//最小歩数を255歩(mapがunsigned char型なので)に設定	

	priority = 0;													//優先度の初期値は0
	
    //maskの意味はstatic_parameter.hを参照
	if( (wall[mypos.x][mypos.y].north & mask) == NOWALL)			//北に壁がなければ
	{
		tmp_priority = get_priority(mypos.x, mypos.y + 1, north);	//優先度を算出
		if(map[mypos.x][mypos.y+1] < little)						//一番歩数が小さい方向を見つける
		{
			little = map[mypos.x][mypos.y+1];						//ひとまず北が歩数が小さい事にする
			*dir = north;											//方向を保存
			priority = tmp_priority;								//優先度を保存
		}
		else if(map[mypos.x][mypos.y+1] == little)					//歩数が同じ場合は優先度から判断する
		{
			if(priority < tmp_priority )							//優先度を評価
			{
				*dir = north;										//方向を更新
				priority = tmp_priority;							//優先度を保存
			}
		}
	}
	
	if( (wall[mypos.x][mypos.y].east & mask) == NOWALL)				//東に壁がなければ
	{
		tmp_priority = get_priority(mypos.x + 1, mypos.y, east);	//優先度を算出
		if(map[mypos.x + 1][mypos.y] < little)						//一番歩数が小さい方向を見つける
		{
			little = map[mypos.x+1][mypos.y];						//ひとまず東が歩数が小さい事にする
			*dir = east;											//方向を保存
			priority = tmp_priority;								//優先度を保存
		}
		else if(map[mypos.x + 1][mypos.y] == little)				//歩数が同じ場合、優先度から判断
		{
			if(priority < tmp_priority)								//優先度を評価
			{
				*dir = east;										//方向を保存
				priority = tmp_priority;							//優先度を保存
			}
		}
	}
	
	if( (wall[mypos.x][mypos.y].south & mask) == NOWALL)			//南に壁がなければ
	{
		tmp_priority = get_priority(mypos.x, mypos.y - 1, south);	//優先度を算出
		if(map[mypos.x][mypos.y - 1] < little)						//一番歩数が小さい方向を見つける
		{
			little = map[mypos.x][mypos.y-1];						//ひとまず南が歩数が小さい事にする
			*dir = south;											//方向を保存
			priority = tmp_priority;								//優先度を保存
		}
		else if(map[mypos.x][mypos.y - 1] == little)				//歩数が同じ場合、優先度で評価
		{
			if(priority < tmp_priority)								//優先度を評価
			{
				*dir = south;										//方向を保存
				priority = tmp_priority;							//優先度を保存
			}
		}
	}
	
	if( (wall[mypos.x][mypos.y].west & mask) == NOWALL)				//西に壁がなければ
	{
		tmp_priority = get_priority(mypos.x - 1, mypos.y, west);	//優先度を算出
		if(map[mypos.x-1][mypos.y] < little)						//一番歩数が小さい方向を見つける
		{
			little = map[mypos.x-1][mypos.y];						//西が歩数が小さい
			*dir = west;											//方向を保存
			priority = tmp_priority;								//優先度を保存
		}
		else if(map[mypos.x - 1][mypos.y] == little)				//歩数が同じ場合、優先度で評価
		{
			*dir = west;											//方向を保存
			priority = tmp_priority;								//優先度を保存
		}
	}

    if(little == 255) {
	unable_to_find_path_to_goal = 1;
	return stop;
    }

	return ( (int)( ( 4 + *dir - mypos.dir) % 4 ) );				//どっちに向かうべきかを返す。
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
	mypos.x = mypos.y = 0;							//座標を初期化
	mypos.dir = north;								//方角を初期化
    set_wall(mypos.x,mypos.y);					//壁をセット

    API_turnLeft();
    API_turnLeft();
	mypos.x = mypos.y = 0;							//座標を初期化
	mypos.dir = south;								//方角を初期化
    set_wall(mypos.x,mypos.y);					//壁をセット

    API_turnLeft();
    API_turnLeft();
	mypos.dir = north;								//方角を初期化

	fast_full_search_adachi_slalom(15,8, 15,8);
	fast_full_search_adachi_slalom(7,0, 15,8);
	fast_full_search_adachi_slalom(15,8, 7,0);
	fast_full_search_adachi_slalom(7,0,7,0);

    API_turnLeft();
    API_turnLeft();
    //*/
	mypos.x = mypos.y = 0;							//座標を初期化
	mypos.dir = north;								//方角を初期化
    API_clearAllColor();

    //get_nextdir(GOAL_X, GOAL_Y,MASK_SECOND,&mypos.dir);

    // 壁と柱のマップを作成
    for(int j = 0; j < MAZESIZE_Y; j++)						//迷路の大きさ分ループ(x座標)
    {
    	for(int i = 0; i < MAZESIZE_X; i++)					//迷路の大きさ分ループ(y座標)
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

    // 壁と柱を表示
    for(int j = MAZESIZE_Y*2; j >= 0; j--)						//迷路の大きさ分ループ(x座標)
    {
    	for(int i = 0; i < MAZESIZE_X*2+1; i++)					//迷路の大きさ分ループ(y座標)
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


    
    // recusionで最速斜めコストマップ作成
	//init_map_naname(GOAL_X,GOAL_Y);											//Mapを初期化する
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

	mypos.x = mypos.y = 0;							//座標を初期化
	mypos.dir = north;								//方角を初期化
    recusion_weight =20;
    color = 'B';
    timer = calculate_max_fast_run_diagonal_time_with_map(GOAL_X, GOAL_Y);
    fprintf(stderr, "\n\nsecond total runtime: %u\n\n", timer);
    fflush(stderr);

	mypos.x = mypos.y = 0;							//座標を初期化
	mypos.dir = north;								//方角を初期化
    recusion_weight =30;
    color = 'C';
    timer = calculate_max_fast_run_diagonal_time_with_map(GOAL_X, GOAL_Y);
    fprintf(stderr, "\n\nthird total runtime: %u\n\n", timer);
    fflush(stderr);

	mypos.x = mypos.y = 0;							//座標を初期化
	mypos.dir = north;								//方角を初期化
    recusion_weight =40;
    color = 'y';
    timer = calculate_max_fast_run_diagonal_time_with_map(GOAL_X, GOAL_Y);
    fprintf(stderr, "\n\nfourth total runtime: %u\n\n", timer);
    fflush(stderr);

	mypos.x = mypos.y = 0;							//座標を初期化
	mypos.dir = north;								//方角を初期化
    recusion_weight =50;
    color = 'o';
    timer = calculate_max_fast_run_diagonal_time_with_map(GOAL_X, GOAL_Y);
    fprintf(stderr, "\n\nfifth total runtime: %u\n\n", timer);
    fflush(stderr);
    /*
    // コストマップを表示
    char str[5];
    unsigned short weight;
    for(int j = 15; j >= 0; j--)						//迷路の大きさ分ループ(x座標)
    {
    	for(int i = 0; i < MAZESIZE_X; i++)					//迷路の大きさ分ループ(y座標)
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
    // 32x32コストマップを表示
    char str[5];
    unsigned short weight;
    for(int j = MAX_Y; j >= 0; j--)						//迷路の大きさ分ループ(x座標)
    {
    	for(int i = 0; i < MAX_X+1; i++)					//迷路の大きさ分ループ(y座標)
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
	mypos.x = mypos.y = 7;							//座標を初期化
	mypos.dir = north;								//方角を初期化
    t_direction fast_dir = get_nextdir(0, 0,MASK_SECOND,&mypos.dir);
	mypos.dir = fast_dir;								//方角を初期化
    */
}
