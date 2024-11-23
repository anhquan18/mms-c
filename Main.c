#include <stdio.h>
#include "API.h"


#define MAZESIZE_X	(16)		//迷路の大きさ(MAZESIZE_X * MAZESIZE_Y)迷路
#define MAZESIZE_Y	(16)		//迷路の大きさ(MAZESIZE_X * MAZESIZE_Y)迷路
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
unsigned char	map[MAZESIZE_X][MAZESIZE_Y];	//歩数マップ
//unsigned int		timer;							//1mSごとにカウントアップされる変数.



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
	                    if(unable_to_find_path_to_goal == 1) {
	                        unable_to_find_path_to_goal = 0;
	                        continue;
	                    }
			    
	                    //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);	//一区画進む
                        API_moveForward();
	                    break;
	                
	                case right:
	                    //rotate(right,1);									//右に曲がって
	                    //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);	//半区画進む
                        API_turnRight();
                        API_moveForward();
	                    break;
	                
	                case left:
	                    //rotate(left,1);										//左に曲がって
	                    //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);	//半区画進む
                        API_turnLeft();
                        API_moveForward();
	                    break;
	                
	                case rear:
			    
	                    //rotate(left,2);										//180ターン
	                    //straight(HALF_SECTION,0,SEARCH_SPEED,SEARCH_SPEED);	//半区画進む
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
	            mypos.dir = glob_nextdir;									//方向を更新


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
	            
	            while(((mypos.x != gx) || (mypos.y != gy)) && (!no_goal_exit_loop_flag)){							//ゴールするまで繰り返す
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



void make_fast_run_map(int x, int y, int mask)	//歩数マップを作成する
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



int get_nextdir_fast_run(int x, int y, int mask, t_direction *dir)	
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




void log(char* text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}

int main(int argc, char* argv[]) {
    log("Running...");
    API_setColor(0, 0, 'G');
    API_setText(0, 0, "abc");
	mypos.x = mypos.y = 0;							//座標を初期化
    set_wall(mypos.x,mypos.y);					//壁をセット

    API_turnLeft();
    API_turnLeft();
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
	mypos.dir = north;								//方角を初期化
    API_clearAllColor();
    get_nextdir(GOAL_X, GOAL_Y,MASK_SECOND,&mypos.dir);

    /*
	mypos.x = mypos.y = 7;							//座標を初期化
	mypos.dir = north;								//方角を初期化
    t_direction fast_dir = get_nextdir(0, 0,MASK_SECOND,&mypos.dir);
	mypos.dir = fast_dir;								//方角を初期化
    */
}
