//#include "StdAfx.h"

#include"astar.h"

//int Castar::moveCost = 0;
//int Castar::nodeNumber = 1;

Castar::Castar()
{
	m_bFind = false;    

	//m_path = NULL;              //存放路径搜索结果
	moveCost = 0;               //路径消耗
	nodeNumber = 0;             //节点个数
}

Castar::~Castar(void)
{
}

void Castar:: ExpandPlanMap()
{
	IplImage* m_tempImg;
	m_tempImg = cvLoadImage("src/aco_planner/src/m_planImg.jpg",0);
	IplImage* m_expandImg=cvCreateImage(cvSize(mapWidth,mapHeight),m_tempImg->depth,m_tempImg->nChannels);
	cvCopy( m_tempImg, m_expandImg, NULL );
	//m_expandImg = m_tempImg;
	CvScalar p;  
	for (int i = 0; i < mapHeight; i++)
	{
		for (int j = 0; j < mapWidth; j++)
		{
			p = cvGet2D(m_tempImg,j,i);//(j,i)
			if(p.val[0]<248)
			{
				cvCircle(m_expandImg,cvPoint(i,j),expandFactor,CV_RGB(0,0,0),-1,-1,0);
			}
		}
	}
	cvSaveImage("src/aco_planner/Debug/m_expandImg.jpg",m_expandImg,0);
	cvReleaseImage(&m_tempImg);
	cvReleaseImage(&m_expandImg);
}

void Castar::InitWalkabilityMap()
{
	ExpandPlanMap();
	IplImage* tempImage;
	tempImage = cvLoadImage("src/aco_planner/Debug/m_expandImg.jpg",0);
	CvScalar p;  
	ofstream outfile1("src/aco_planner/Debug/GrayMap.txt");  
	ofstream outfile2("src/aco_planner/Debug/WalkabilityMap.txt");  
	for (unsigned int i = 0; i < mapHeight; i++)
	{
		for (unsigned int j = 0; j < mapWidth; j++)
		{
			p = cvGet2D(tempImage,j,i);//(j,i)  
            outfile1<<p.val[0]<<" ";  
			if(p.val[0]>247)
			{
				walkability[j][i] = walkable;
			}
			else
			{
				walkability[j][i] = unwalkable;
			}
			outfile2<<walkability[j][i]<<" ";  
		}
		outfile1<<endl;  
		outfile2<<endl;  
	}
	outfile1.close();
	outfile2.close();
	/*cvNamedWindow("tempImage",CV_WINDOW_AUTOSIZE);
	cvShowImage("tempImage",tempImage);  
	cvWaitKey(0);*/
	cvSaveImage("src/aco_planner/Debug/tempImage.jpg",tempImage,0);
	cvReleaseImage(&tempImage);  
}

bool Castar::IsFindPath()
{
    return m_bFind;
}

int Castar::FindPath (Node startNode, Node endNode)
{	
	//Create needed arrays

	int Fcost[mapWidth*mapHeight+2];	//1d array to store F cost of a cell on the open list （F = G + H）
	//总的代价值=历史代价+启发代价
	int Gcost[mapHeight+1][mapWidth+1]; //2d array to store G cost for each cell.
	int Hcost[mapWidth*mapHeight+2];	//1d array to store H cost of a cell on the open list

	int openList[mapWidth*mapHeight+2]; //1 dimensional array holding ID# of open list items
	int whichList[mapHeight][mapWidth];  //2 dimensional array used to record whether a cell is on the open list or on the closed list.
	int openX[mapWidth*mapHeight+2]; //1d array stores the x location of an item on the open list
	int openY[mapWidth*mapHeight+2]; //1d array stores the y location of an item on the open list
	int parentX[mapHeight+1][mapWidth+1]; //2d array to store parent of each cell (x)
	int parentY[mapHeight+1][mapWidth+1]; //2d array to store parent of each cell (y)

	int parentXval=0, parentYval=0,
	a=0, b=0, m=0, u=0, v=0, temp=0, corner=0, numberOfOpenListItems=0,
	addedGCost=0, tempGcost = 0, pathStatus = 0,
	tempx, tempy, pathX, pathY,
	newOpenListItemID = 0;

	for (int i=0;i<mapWidth;i++)
	{
		for (int j=0;j<mapHeight;j++)
		{				
				whichList[i][j] = 0;				
		}
	}

//1. Convert location data (in pixels) to coordinates in the walkability array.
	int startX = startNode.y + 0.5, startY = startNode.x + 0.5;
	int targetX = endNode.y + 0.5, targetY = endNode.x + 0.5;
	/*int startX = startingX;
	int startY = startingY;	
	targetX = targetX/tileSize;
	targetY = targetY/tileSize;*/

//2.Quick Path Checks: Under the some circumstances no path needs to
//	be generated ...

	//	If start square is unexist, return that its putin is error.
	if (startX<0 || startY<0 || startX>=mapHeight || startY>=mapWidth)
	{	
		m_bFind = false;
		return errorputin;
	}

	//	If target square is unexist, return that its putin is error.
	if (targetX<0 || targetY<0 || targetX>=mapHeight || targetY>=mapWidth)
	{	
		m_bFind = false;
		return errorputin;
	}

	//	If start square is unwalkable, return that its putin is error.
	if (walkability[startX][startY] == unwalkable)
	{	
		m_bFind = false;
		return errorputin;
	}
	
	//	If target square is unwalkable, return that its putin is error.
	if (walkability[targetX][targetY] == unwalkable)
	{	
		m_bFind = false;
		return errorputin;
	}

	//	If starting location and target are in the same location...
	if (startX == targetX && startY == targetY )
	{
		//输出路径
		moveCost = 0;
		nodeNumber = 0;
		m_bFind = true;
		return found;
	}

//3.Reset some variables that need to be cleared
/*	if (onClosedList > 1000000) //reset whichList occasionally ?
	{
		for (int x = 0; x < mapWidth;x++) 
		{
			for (int y = 0; y < mapHeight;y++)
				whichList[x][y] = 0;
		}
		onClosedList = 10;	
	}
	onClosedList = onClosedList+2; //changing the values of onOpenList and onClosedlist is faster than redimming whichList array
	onOpenList = onClosedList-1;
*/
	nodeNumber  = 0;//i.e, = 0
	moveCost = 0;
	Gcost[startX][startY] = 0; //reset starting square's G value to 0

//4.Add the starting location to the open list of squares to be checked.
	numberOfOpenListItems = 1;
	openList[1] = 1;//assign it as the top (and currently only) item in the open list, which is maintained as a binary heap (explained below)
	openX[1] = startX;
	openY[1] = startY;

//5.Do the following until a path is found or deemed nonexistent.
	do
	{
		//寻找开启列表中F值最低的栅格，作为当前格。
		//6.If the open list is not empty, take the first cell off of the list.
		//	This is the lowest F cost cell on the open list.
		if (numberOfOpenListItems != 0)
		{
			//cout<<numberOfOpenListItems<<endl;
			//7. Pop the first item off the open list.
			parentXval = openX[openList[1]];
			parentYval = openY[openList[1]]; //record cell coordinates of the item

			whichList[parentXval][parentYval] = onClosedList;//add the item to the closed list

		//	Open List = Binary Heap: Delete this item from the open list, which
		//  is maintained as a binary heap. 

			numberOfOpenListItems -= 1;//reduce number of open list items by 1	
		
		//	Delete the top item in binary heap and reorder the heap, with the lowest F cost item rising to the top.
			openList[1] = openList[numberOfOpenListItems+1];//move the last item in the heap up to slot #1
			v = 1;

		//	Repeat the following until the new item in slot #1 sinks to its proper spot in the heap.树的排序
			do
			{
				u = v;		
				if (2*u+1 <= numberOfOpenListItems) //if both children exist
				{
	 				//Check if the F cost of the parent is greater than each child.
					//Select the lowest of the two children.
					if (Fcost[openList[u]] >= Fcost[openList[2*u]]) 
						v = 2*u;
					if (Fcost[openList[v]] >= Fcost[openList[2*u+1]]) 
						v = 2*u+1;		
				}
				else
				{
					if (2*u <= numberOfOpenListItems) //if only child #1 exists
					{
	 				//Check if the F cost of the parent is greater than child #1	
						if (Fcost[openList[u]] >= Fcost[openList[2*u]]) 
							v = 2*u;
					}
				}

				if (u != v) //if parent's F is > one of its children, swap them
				{
					temp = openList[u];
					openList[u] = openList[v];
					openList[v] = temp;			
				}
				else
					break; //otherwise, exit loop		
			}
			while (1);//reorder the binary heap

	//7.Check the adjacent squares. (Its "children" -- these path children
			//	are similar, conceptually, to the binary heap children mentioned
			//	above, but don't confuse them. They are different. Path children
			//	are portrayed in Demo 1 with grey pointers pointing toward
			//	their parents.) Add these adjacent child squares to the open list
			//	for later consideration if appropriate (see various if statements
			//	below).
			for (b = parentYval-1; b <= parentYval+1; b++)
			{
				for (a = parentXval-1; a <= parentXval+1; a++)
				{
					//	If not off the map (do this first to avoid array out-of-bounds errors)
					if (a != -1 && b != -1 && a != mapHeight && b != mapWidth)
					{
						//	If not already on the closed list (items on the closed list have
						//	already been considered and can now be ignored).			
						if (whichList[a][b] != onClosedList)    //子栅格不在关闭列表中
						{ 
							//	If not a wall/obstacle square.
							if (walkability[a][b] == walkable)  //子栅格可以通行
							{ 
								//	Don't cut across corners
								corner = walkable;

								if (a == parentXval-1)    // 对四个辅方向进行可通行性判断                         
								{
									if (b == parentYval-1)
									{
										if (walkability[parentXval-1][parentYval] == unwalkable
											|| walkability[parentXval][parentYval-1] == unwalkable) //左上方
											corner = unwalkable;
									}
									else if (b == parentYval+1)
									{
										if (walkability[parentXval][parentYval+1] == unwalkable
											|| walkability[parentXval-1][parentYval] == unwalkable) //右上方
											corner = unwalkable; 
									}
								}
								else if (a == parentXval+1)
								{
									if (b == parentYval-1)
									{
										if (walkability[parentXval][parentYval-1] == unwalkable 
											|| walkability[parentXval+1][parentYval] == unwalkable)  //左下方
											corner = unwalkable;
									}
									else if (b == parentYval+1)
									{
										if (walkability[parentXval+1][parentYval] == unwalkable 
											|| walkability[parentXval][parentYval+1] == unwalkable)  // 右下方
											corner = unwalkable; 
									}
								}

								if (corner == walkable) 
								{
									//	If not already on the open list, add it to the open list.栅格不在开启列表中的情况			
									if (whichList[a][b] != onOpenList) 
									{	
										//Create a new open list item in the binary heap.
										newOpenListItemID = newOpenListItemID + 1; //each new item has a unique ID #
										m = numberOfOpenListItems + 1;
										openList[m] = newOpenListItemID;//place the new open list item (actually, its ID#) at the bottom of the heap
										openX[newOpenListItemID] = a;
										openY[newOpenListItemID] = b;//record the x and y coordinates of the new item

										//Figure out its G cost
										if (abs(a-parentXval) == 1 && abs(b-parentYval) == 1)
											addedGCost = 14;//cost of going to diagonal squares	
										else	
											addedGCost = 10;//cost of going to non-diagonal squares				
										Gcost[a][b] = Gcost[parentXval][parentYval] + addedGCost;

										//Figure out its H and F costs and parent
										Hcost[openList[m]] = 10*(abs(a - targetX) + abs(b - targetY));
										Fcost[openList[m]] = Gcost[a][b] + Hcost[openList[m]];
										parentX[a][b] = parentXval; 
										parentY[a][b] = parentYval;	

										//Move the new open list item to the proper place in the binary heap.
										//Starting at the bottom, successively compare to parent items,
										//swapping as needed until the item finds its place in the heap
										//or bubbles all the way to the top (if it has the lowest F cost).
										while (m != 1) //While item hasn't bubbled to the top (m=1)	
										{
											//Check if child's F cost is < parent's F cost. If so, swap them.	
											if (Fcost[openList[m]] <= Fcost[openList[m/2]])
											{
												temp = openList[m/2];
												openList[m/2] = openList[m];
												openList[m] = temp;
												m = m/2;
											}
											else
												break;
										}
										numberOfOpenListItems = numberOfOpenListItems+1;//add one to the number of items in the heap

										//Change whichList to show that the new item is on the open list.
										whichList[a][b] = onOpenList;
									}

								//8.If adjacent cell is already on the open list, check to see if this 
								//	path to that cell from the starting location is a better one. 
								//	If so, change the parent of the cell and its G and F costs.	
									else //If whichList(a,b) = onOpenList，栅格已经在开启列表的情况
									{
										//Figure out the G cost of this possible new path
										if (abs(a-parentXval) == 1 && abs(b-parentYval) == 1)
											addedGCost = 14;//cost of going to diagonal tiles	
										else	
											addedGCost = 10;//cost of going to non-diagonal tiles				
										tempGcost = Gcost[parentXval][parentYval] + addedGCost;
		
										//If this path is shorter (G cost is lower) then change
										//the parent cell, G cost and F cost. 		
										if (tempGcost < Gcost[a][b]) //if G cost is less,
										{
											parentX[a][b] = parentXval; //change the square's parent
											parentY[a][b] = parentYval;
											Gcost[a][b] = tempGcost;//change the G cost			

											//Because changing the G cost also changes the F cost, if
											//the item is on the open list we need to change the item's
											//recorded F cost and its position on the open list to make
											//sure that we maintain a properly ordered open list.
											for (int x = 1; x <= numberOfOpenListItems; x++) //look for the item in the heap
											{
												if (openX[openList[x]] == a && openY[openList[x]] == b) //item found
												{
													Fcost[openList[x]] = Gcost[a][b] + Hcost[openList[x]];//change the F cost
				
													//See if changing the F score bubbles the item up from it's current location in the heap
													m = x;
													while (m != 1) //While item hasn't bubbled to the top (m=1)	
													{
														//Check if child is < parent. If so, swap them.	
														if (Fcost[openList[m]] < Fcost[openList[m/2]])
														{
															temp = openList[m/2];
															openList[m/2] = openList[m];
															openList[m] = temp;
															m = m/2;
														}
														else
															break;
													} 
													break; //exit for x = loop
												} //If openX(openList(x)) = a
											} //For x = 1 To numberOfOpenListItems
										}//If tempGcost < Gcost(a,b)
									}//else If whichList(a,b) = onOpenList	
								}//If not cutting a corner
							}//If not a wall/obstacle square.
						}//If not already on the closed list 
					}//If not off the map
				}//for (a = parentXval-1; a <= parentXval+1; a++){
			}//for (b = parentYval-1; b <= parentYval+1; b++){
		}//if (numberOfOpenListItems != 0)
		//9.If open list is empty then there is no path.	
		else
		{
			pathStatus = nonexistent; 
			//cout<<"Finish1"<<endl;
			break;
		}  
		//If target is added to open list then path has been found.
		if (whichList[targetX][targetY] == onOpenList)
		{
			pathStatus = found; 
			/*pathStatus = 1;*/
			//cout<<"Finish2"<<endl;
			break;
		}
	}
	while (1);//Do until path is found or deemed nonexistent
	//10.Save the path if it exists.
	if (pathStatus == found)
	{
		//a.Working backwards from the target to the starting location by checking
		//	each cell's parent, figure out the length of the path.
		pathX = targetX; 
		pathY = targetY;
		path[nodeNumber][0] = pathX;
		path[nodeNumber][1] = pathY;
//		cout<<pathX<<","<<pathY<<endl;
//		walkability[pathX][pathY] = 2;
		m_path.clear();
		Node node;
		node.x = pathY;
		node.y = pathX;
		m_path.push_back(node);
		nodeNumber = nodeNumber + 1;
		do
		{
			//Look up the parent of the current cell.
			tempx = parentX[pathX][pathY];
			tempy = parentY[pathX][pathY];
			//Figure out the path length
			if((tempx == pathX) || (tempy == pathY))
			{
				moveCost += 1.0;
			}
			else
			{
				moveCost += 1.414;
			}

			pathX = tempx;
			pathY = tempy;
			
//			walkability[pathX][pathY] = 2;
//			cout<<pathX<<","<<pathY<<endl;
			//Figure out the path lnodeNumber
			nodeNumber = nodeNumber + 1;
			path[nodeNumber][0] = pathX;
			path[nodeNumber][1] = pathY;
			node.x = pathY;
			node.y = pathX;
			m_path.push_back(node);
		}
		while (pathX != startX || pathY != startY);
		//m_path.pop_back();

		//cout<<nodeNumber<<endl;
		//cout<<moveCost<<endl;
		//cout<<"Finish3"<<endl;
		moveCost = moveCost * singleGridSize;
		m_bFind = true;
	}

	if (m_bFind)
	{
		//GetFinalShortestPath();
	}
	return pathStatus;
}

void Castar::ShowMapwithPath()
{
	if(m_bFind)
	{
		IplImage* tempImage;
		tempImage = cvLoadImage("m_planImg.jpg",-1);
		uchar* data=(uchar *)tempImage->imageData; 
		int step = tempImage->widthStep/sizeof(uchar);  
		int chanels = tempImage->nChannels;
		for(int i=0;i<m_path.size();i++)
		{
			data[path[i][0]*step+path[i][1]*chanels+0] = 0; //b  
			data[path[i][0]*step+path[i][1]*chanels+1] = 0; //g
			data[path[i][0]*step+path[i][1]*chanels+2] = 255; //r 
		}
		cvNamedWindow("tempImage",CV_WINDOW_AUTOSIZE);
		cvShowImage("tempImage",tempImage);  
		cvWaitKey(0);
		cvSaveImage("tempImage.jpg",tempImage,0);
		cvReleaseImage(&tempImage);
	}

}

vector<Node>* Castar::GetPathVector()
    {
		return &m_path; 		
    }

double Castar::GetmoveCost()
{
	return moveCost;
}
