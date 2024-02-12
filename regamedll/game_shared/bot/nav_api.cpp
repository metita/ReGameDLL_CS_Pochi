#include "precompiled.h"

ConnectInfoList TheConnectInfoList;

ConnectInfoData* AddConnectInfoList()
{
	ConnectInfoData *data = new ConnectInfoData;
	
    // Init anything to avoid weird things
    data->path = new ConnectInfo_api[MAX_PATH_LENGTH_API];
    data->update = gpGlobals->time;
    data->length = 0;
    data->index = 0;

	TheConnectInfoList.push_front(data);
	return data;
}

bool RemoveConnectInfoList(ConnectInfoData *data)
{
	if(!data)
		return false;

	// loop till gone
	for(auto iter = TheConnectInfoList.begin(); iter != TheConnectInfoList.end(); iter++)
	{
		if(*iter == data)
		{
			TheConnectInfoList.remove(data);
			
			delete[] data->path;
			delete data;

			data = nullptr;
			return true;
		}
	}

	// update anyway
	data = nullptr;
	return false;
}

// called in DestroyNavigationMap
void DestroyConnectInfoList()
{
	// remove each element of the list and delete them
	while (!TheConnectInfoList.empty())
	{
		ConnectInfoData *path = TheConnectInfoList.front();
		TheConnectInfoList.pop_front();
		delete path;
	}
}

void BuildTrivialPath_api(ConnectInfoData *data, ConnectInfo_api *path, CNavArea *startArea, const Vector *start, const Vector *goal)
{
	path[0].area = startArea;
	path[0].pos = *start;
	path[0].pos.z = startArea->GetZ(start);
	path[0].ladder = nullptr;
	path[0].how = NUM_TRAVERSE_TYPES;

	path[1].area = startArea;
	path[1].pos = *goal;
	path[1].pos.z = startArea->GetZ(goal);
	path[1].ladder = nullptr;
	path[1].how = NUM_TRAVERSE_TYPES;

	data->index = 1;
	data->length = 2;
}

bool ComputePathPositions_api(ConnectInfo_api *path, int &length)
{
	if (length == 0)
		return false;

	// start in first area's center
	path[0].pos = *path[0].area->GetCenter();
	path[0].ladder = nullptr;
	path[0].how = NUM_TRAVERSE_TYPES;

	for (int i = 1; i < length; i++)
	{
		const ConnectInfo_api *from = &path[i - 1];
		ConnectInfo_api *to = &path[i];

		// walk along the floor to the next area
		if (to->how <= GO_WEST)
		{
			to->ladder = nullptr;

			// compute next point, keeping path as straight as possible
			from->area->ComputeClosestPointInPortal(to->area, (NavDirType)to->how, &from->pos, &to->pos);

			// move goal position into the goal area a bit
			// how far to "step into" an area - must be less than min area size
			const float stepInDist = 5.0f;
			AddDirectionVector(&to->pos, (NavDirType)to->how, stepInDist);

			// we need to walk out of "from" area, so keep Z where we can reach it
			to->pos.z = from->area->GetZ(&to->pos);

			// if this is a "jump down" connection, we must insert an additional point on the path
			if (to->area->IsConnected(from->area, NUM_DIRECTIONS) == false)
			{
				// this is a "jump down" link
				// compute direction of path just prior to "jump down"
				Vector2D dir;
				DirectionToVector2D((NavDirType)to->how, &dir);

				// shift top of "jump down" out a bit to "get over the ledge"
				const float pushDist = 25.0f; // 75.0f;
				to->pos.x += pushDist * dir.x;
				to->pos.y += pushDist * dir.y;

				// insert a duplicate node to represent the bottom of the fall
				if (length < MAX_PATH_LENGTH_API - 1)
				{
					// copy nodes down
					for (int j = length; j > i; j--)
						path[j] = path[j - 1];

					// path is one node longer
					length++;

					// move index ahead into the new node we just duplicated
					i++;

					path[i].pos.x = to->pos.x + pushDist * dir.x;
					path[i].pos.y = to->pos.y + pushDist * dir.y;

					// put this one at the bottom of the fall
					path[i].pos.z = to->area->GetZ(&path[i].pos);
				}
			}
		}
		// to get to next area, must go up a ladder
		else if (to->how == GO_LADDER_UP)
		{
			// find our ladder
			const NavLadderList *list = from->area->GetLadderList(LADDER_UP);
			NavLadderList::const_iterator iter;
			for (iter = list->begin(); iter != list->end(); iter++)
			{
				CNavLadder *ladder = (*iter);

				// can't use "behind" area when ascending...
				if (ladder->m_topForwardArea == to->area || ladder->m_topLeftArea == to->area || ladder->m_topRightArea == to->area)
				{
					to->ladder = ladder;
					to->pos = ladder->m_bottom;
					AddDirectionVector(&to->pos, ladder->m_dir, HalfHumanWidth * 2.0f);
					break;
				}
			}

			if (iter == list->end())
				return false;
		}
		// to get to next area, must go down a ladder
		else if (to->how == GO_LADDER_DOWN)
		{
			// find our ladder
			const NavLadderList *list = from->area->GetLadderList(LADDER_DOWN);
			NavLadderList::const_iterator iter;
			for (iter = list->begin(); iter != list->end(); iter++)
			{
				CNavLadder *ladder = (*iter);

				if (ladder->m_bottomArea == to->area)
				{
					to->ladder = ladder;
					to->pos = ladder->m_top;
					AddDirectionVector(&to->pos, OppositeDirection(ladder->m_dir), HalfHumanWidth * 2.0f);
					break;
				}
			}

			if (iter == list->end())
				return false;
		}
	}

	return true;
}

ConnectInfoData* ComputePath_api(ConnectInfoData *data, CNavArea *startArea, const Vector *start, CNavArea *goalArea, const Vector *goal, RouteType route)
{
	// NOTE: This is a cheap way to do this, cuz I only reassembled the base code, needs extra functions to check if it can TRULY reach the path
	// should provide start area
	if(!startArea)
		return data;
	
	// MAX_PATH_LENGTH is 256
	// this nooby way could be better (I guess), free feel to pr to improve this code
	// even if bots already does this, is this smart enough?
	
	if(!data)
    {
	    // If no pointer is provided, add one
    	data = AddConnectInfoList();
    }
    else
    {
        // too fast
        if(data->update > gpGlobals->time)
            return data;    
    }
    
    // Generate one the next time (same as cs_bot_pathfind.cpp)
    data->update = gpGlobals->time + RANDOM_FLOAT(0.4f, 0.6f);
    ConnectInfo_api *path = data->path;

	Vector vecEndPosition = *goal;

	// make sure path end position is on the ground
	if (goalArea)
		vecEndPosition.z = goalArea->GetZ(&vecEndPosition);
	else
		GetGroundHeight(&vecEndPosition, &vecEndPosition.z);

	// if we are already in the goal area, build trivial path
	if (startArea == goalArea)
	{
		BuildTrivialPath_api(data, path, startArea, start, goal);
		return data;
	}

	// Compute shortest path to goal
	CNavArea *closestArea = nullptr;

	// HACK: use hostage path cost for this
	HostagePathCost pathCost;

	bool pathToGoalExists = NavAreaBuildPath(startArea, goalArea, goal, pathCost, &closestArea);
	CNavArea *effectiveGoalArea = (pathToGoalExists) ? goalArea : closestArea;

	// Build path by following parent links
	// get count
	int count = 0;
	CNavArea *area;
	for (area = effectiveGoalArea; area; area = area->GetParent())
	{
		count++;
	}

	// save room for endpoint
	if (count > MAX_PATH_LENGTH_API - 1)
		count = MAX_PATH_LENGTH_API - 1;

	if (count == 0)
		return data;

	if (count == 1)
	{
		BuildTrivialPath_api(data, path, startArea, start, goal);
		return data;
	}

	// build path
	data->length = count;
	for (area = effectiveGoalArea; count && area; area = area->GetParent())
	{
		count--;
		path[count].area = area;
		path[count].how = area->GetParentHow();
	}

	// compute path positions to this
	if(!ComputePathPositions_api(path, data->length))
	{
		// idk, it stills builds a path but should be returning false lol
	}

	// append path end position
	path[data->length].area = effectiveGoalArea;
	path[data->length].pos = vecEndPosition;
	path[data->length].ladder = nullptr;
	path[data->length].how = NUM_TRAVERSE_TYPES;

	data->length++;

	return data;
}
