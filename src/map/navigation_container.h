//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

// Modified: Copyright (c) 2023 AirSkyBoat Dev Team
// Based On: NavMeshTesterTool.cpp
// Note    : This class is heavily pulls from the NavMeshTesterTool.cpp from the Recast Demo project.
//         : It is intended to contain a dtNavMesh and perform any pathfinding queries.

#ifndef _NAVIGATION_CONTAINER_H_
#define _NAVIGATION_CONTAINER_H_

#include <map>
#include <memory>
#include <vector>

#include <DetourCommon.h>
#include <DetourMath.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>
#include <Recast.h>

#include "common/logging.h"
#include "common/mmo.h"

enum NavPolyAreas
{
    POLYAREA_GROUND,
    POLYAREA_WATER,
    POLYAREA_ROAD,
    POLYAREA_DOOR,
    POLYAREA_GRASS,
    POLYAREA_JUMP
};

enum NavPolyFlags
{
    POLYFLAGS_WALK     = 0x01,  // Ability to walk (ground, grass, road)
    POLYFLAGS_SWIM     = 0x02,  // Ability to swim (water), some mobs reset when they hit the water
    POLYFLAGS_DOOR     = 0x04,  // Ability to move through doors.
    POLYFLAGS_JUMP     = 0x08,  // Ability to jump.
    POLYFLAGS_DISABLED = 0x10,  // Disabled polygon
    POLYFLAGS_ALL      = 0xFFFF // All abilities.
};

struct NavMeshSetHeader
{
    int             magic;
    int             version;
    int             numTiles;
    dtNavMeshParams params;
};

struct NavMeshTileHeader
{
    dtTileRef tileRef;
    int       dataSize;
};

namespace Navigation
{
    constexpr float    smallPolyPickExt[3]  = { 0.5f, 1.0f, 0.5f };
    constexpr float    polyPickExt[3]       = { 5.0f, 10.0f, 5.0f };
    constexpr float    skinnyPolyPickExt[3] = { 0.01f, 10.0f, 0.01f };
    constexpr float    bigPolyPickExt[3]    = { 30.0f, 60.0f, 30.0f };
    constexpr float    verticalLimit        = 0.25f;
    static const float PATH_STEP_SIZE       = 3.0f;
    static const float PATH_SLOP            = 0.1f;
    static const int   MAX_POLYS            = 256;
    static const int   MAX_SMOOTH           = 2048;
    static const int   MAX_STEER_POINTS     = 5;
    static const int   NAVMESHSET_MAGIC     = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; // 'MSET'
    static const int   NAVMESHSET_VERSION   = 1;

    void detourError(uint32 status);
    void detourToGamePosition(const position_t* t_dtPos, float* t_pos);
    void detourToGamePosition(float* t_pos);
    void detourToGamePosition(position_t* t_dtPos);
    void detourToGamePosition(position_t& t_pos, const float* t_dtPos);
    void gameToDetourPosition(const position_t* t_pos, float* t_dtPos);
    void gameToDetourPosition(float* t_pos);
    void gameToDetourPosition(position_t* t_pos);
    bool dtGetSteerTarget(dtNavMeshQuery* navQuery, const float* startPos, const float* endPos, const float minTargetDist,
                          const dtPolyRef* path, const int pathSize,
                          float* steerPos, unsigned char& steerPosFlag, dtPolyRef& steerPosRef,
                          float* outPoints = 0, int* outPointCount = 0);
    bool dtInRange(const float* v1, const float* v2, const float r, const float h);
    int  dtFixupCorridor(dtPolyRef* path, const int npath, const int maxPath, const dtPolyRef* visited, const int nvisited);
    int  dtFixupShortcuts(dtPolyRef* path, int npath, dtNavMeshQuery* navQuery);
} // namespace Navigation

typedef std::map<NavPolyAreas, float> AreaCostMap;
typedef unsigned char                 PathFlags;

class CNavigationContainer
{
public:
    CNavigationContainer(uint16 zoneID);
    ~CNavigationContainer();

    //-- Accessors
    float getAgentRadius();
    float getAreaCost(NavPolyAreas t_area);
    void  setAgentRadius(float t_radius);
    void  setAreaCost(NavPolyAreas t_area, float t_cost);

    //-- Members
    bool isInWater(const position_t& t_pos);
    bool isValidPosition(const position_t& t_pos);
    bool isInRange(const position_t& t_start, const position_t& t_end, float t_distance, float t_h);
    auto findStraightPath(const position_t& t_start, const position_t& t_end,
                          unsigned short includePolyFlags = POLYFLAGS_ALL ^ POLYFLAGS_DISABLED,
                          unsigned short excludePolyFlags = 0) -> std::vector<pathpoint_t>;
    auto findSmoothPath(const position_t& t_start, const position_t& t_end,
                        unsigned short includePolyFlags = POLYFLAGS_ALL ^ POLYFLAGS_DISABLED,
                        unsigned short excludePolyFlags = 0) -> std::vector<pathpoint_t>;
    auto findRandomPosition(const position_t& t_around, float t_radius,
                            unsigned short includePolyFlags = POLYFLAGS_ALL ^ POLYFLAGS_DISABLED,
                            unsigned short excludePolyFlags = 0) -> pathpoint_t;
    bool findClosestValidPoint(const position_t& t_start, float* t_point);
    bool findFurthestValidPoint(const position_t& t_start, const position_t& t_end, float* t_point);
    bool raycast(const position_t& t_start, const position_t& t_end);
    void snapToMesh(position_t& t_pos, float t_targetY, bool t_force = false);

    //-- Util
    bool isMeshLoaded();
    bool loadFromFile(std::string const& t_filename);
    void reloadMesh();

private:
    float       m_agentRadius;
    AreaCostMap m_areaCosts = {
        { POLYAREA_GROUND, 1.0f },
        { POLYAREA_WATER, 8.0f },
        { POLYAREA_ROAD, 1.0f },
        { POLYAREA_DOOR, 2.0f },
        { POLYAREA_GRASS, 1.5f },
        { POLYAREA_JUMP, 3.0f },
    };
    std::string m_meshFilename;

    //-- Detour Variables
    dtNavMesh*     m_navMesh;
    dtNavMeshQuery m_query;
    dtQueryFilter  m_filter;
    dtStatus       m_pathFindStatus;
    dtPolyRef      m_startRef;
    dtPolyRef      m_endRef;
    dtPolyRef      m_polys[Navigation::MAX_POLYS];
    dtPolyRef      m_straightPathPolys[Navigation::MAX_POLYS];

    //-- Pathfinding Variables
    float     m_pickExtents[3];
    int       m_npolys = 0;
    int       m_nstraightPath;
    float     m_straightPath[Navigation::MAX_POLYS * 3];
    PathFlags m_straightPathFlags[Navigation::MAX_POLYS];
    int       m_nsmoothpath = 0;
    float     m_smoothPath[Navigation::MAX_SMOOTH * 3];
    float     m_spos[3];
    float     m_epos[3];
    bool      m_hitResult = false;
    float     m_hitPos[3];
    float     m_hitNormal[3];
    int       m_pathIterNum = 0;

    void resetFilter();
    void applyFilter(unsigned short includePolyFlags, unsigned short excludePolyFlags);
};

#endif
