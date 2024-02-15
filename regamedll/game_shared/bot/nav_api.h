/*
*
*   This program is free software; you can redistribute it and/or modify it
*   under the terms of the GNU General Public License as published by the
*   Free Software Foundation; either version 2 of the License, or (at
*   your option) any later version.
*
*   This program is distributed in the hope that it will be useful, but
*   WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*   General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software Foundation,
*   Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
*   In addition, as a special exception, the author gives permission to
*   link the code of this program with the Half-Life Game Engine ("HL
*   Engine") and Modified Game Libraries ("MODs") developed by Valve,
*   L.L.C ("Valve").  You must obey the GNU General Public License in all
*   respects for all of the code used other than the HL Engine and MODs
*   from Valve.  If you modify this file, you may extend this exception
*   to your version of the file, but you are not obligated to do so.  If
*   you do not wish to do so, delete this exception statement from your
*   version.
*
*/
#pragma once

// Paths API
// optimizations are welcome
enum { MAX_PATH_LENGTH_API = 256 };
struct ConnectInfo_api
{
	CNavArea *area;				// the area along the path
	NavTraverseType how;		// how to enter this area from the previous one
	Vector pos;					// our movement goal position at this point in the path
	const CNavLadder *ladder;	// if "how" refers to a ladder, this is it
};

struct ConnectInfoData
{
	CBaseEntity* entity;		// path entity
	ConnectInfo_api *path;		// path pointer
	CNavArea *currentArea;		// path's current area
	Vector currentGoal;			// path current goal (fast access)
	int length;					// path length
	int index;					// path current index
    float update;               // path next update
	float update_min;			// path min delay update
	float update_max;			// path max delay update
};

typedef std::list<ConnectInfoData *> ConnectInfoList;
extern ConnectInfoList TheConnectInfoList;

// 0.4 and 0.6 are taken from cs_bot_pathfind respectively
ConnectInfoData* AddConnectInfoList(CBaseEntity *entity, float update_min = 0.4f, float update_max = 0.6f);
ConnectInfoData* GetConnectInfoList(CBaseEntity *entity);
bool RemoveConnectInfoList(CBaseEntity *entity);
void DestroyConnectInfoList();

void BuildTrivialPath_api(ConnectInfoData *data, ConnectInfo_api *path, CNavArea *startArea, const Vector *start, const Vector *goal);
bool ComputePathPositions_api(ConnectInfo_api *path, int &length);

ConnectInfoData* ComputePath(CBaseEntity *entity, ConnectInfoData *data, CNavArea *startArea, const Vector *start, CNavArea *goalArea, const Vector *goal, RouteType route);

bool UpdatePathMovement(CBaseEntity *entity, ConnectInfoData *data, float tolerance, bool check2D);