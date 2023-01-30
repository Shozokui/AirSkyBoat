#include "navigation_container.h"

namespace Navigation
{
    void detourError(uint32 status)
    {
        if (status & DT_WRONG_MAGIC)
        {
            ShowError("Detour: Input data is not recognized.");
        }
        else if (status & DT_WRONG_VERSION)
        {
            ShowError("Detour: Input data is in wrong version.");
        }
        else if (status & DT_OUT_OF_MEMORY)
        {
            ShowError("Detour: Operation ran out of memory.");
        }
        else if (status & DT_INVALID_PARAM)
        {
            ShowError("Detour: An input parameter was invalid.");
        }
        else if (status & DT_BUFFER_TOO_SMALL)
        {
            ShowError("Detour: Result buffer for the query was too small to store all results.");
        }
        else if (status & DT_OUT_OF_NODES)
        {
            ShowError("Detour: Query ran out of nodes during search.");
        }
        else if (status & DT_PARTIAL_RESULT)
        {
            ShowError("Detour: Query did not reach the end location, returning best guess.");
        }
        else if (status & DT_ALREADY_OCCUPIED)
        {
            ShowError("Detour: A tile has already been assigned to the given x,y coordinate");
        }
    }

    void detourToGamePosition(const position_t* t_dtPos, float* t_pos)
    {
        float y = t_dtPos->y;
        float z = t_dtPos->z;

        t_pos[0] = t_dtPos->x;
        t_pos[1] = y * -1;
        t_pos[2] = z * -1;
    }

    void detourToGamePosition(float* t_pos)
    {
        float y  = t_pos[1];
        float z  = t_pos[2];
        t_pos[1] = y * -1;
        t_pos[2] = z * -1;
    }

    void detourToGamePosition(position_t* t_pos)
    {
        float y  = t_pos->y;
        float z  = t_pos->z;
        t_pos->y = y * -1;
        t_pos->z = z * -1;
    }

    void detourToGamePosition(position_t& t_pos, const float* t_dtPos)
    {
        float y = t_dtPos[1];
        float z = t_dtPos[2];
        t_pos.x = t_dtPos[0];
        t_pos.y = y * -1;
        t_pos.z = z * -1;
    }

    void gameToDetourPosition(const position_t* t_pos, float* t_dtPos)
    {
        float y    = t_pos->y;
        float z    = t_pos->z;
        t_dtPos[0] = t_pos->x;
        t_dtPos[1] = y * -1;
        t_dtPos[2] = z * -1;
    }

    void gameToDetourPosition(float* t_pos)
    {
        float y  = t_pos[1];
        float z  = t_pos[2];
        t_pos[1] = y * -1;
        t_pos[2] = z * -1;
    }

    void gameToDetourPosition(position_t* t_pos)
    {
        float y  = t_pos->y;
        float z  = t_pos->z;
        t_pos->y = y * -1;
        t_pos->z = z * -1;
    }

    bool dtGetSteerTarget(dtNavMeshQuery* navQuery, const float* startPos, const float* endPos,
                          const float      minTargetDist,
                          const dtPolyRef* path, const int pathSize,
                          float* steerPos, unsigned char& steerPosFlag, dtPolyRef& steerPosRef,
                          float* outPoints, int* outPointCount)
    {
        static const int MAX_STEER_POINTS = 3;
        float            steerPath[MAX_STEER_POINTS * 3];
        unsigned char    steerPathFlags[MAX_STEER_POINTS];
        dtPolyRef        steerPathPolys[MAX_STEER_POINTS];
        int              nsteerPath = 0;
        navQuery->findStraightPath(startPos, endPos, path, pathSize,
                                   steerPath, steerPathFlags, steerPathPolys, &nsteerPath, MAX_STEER_POINTS);
        if (!nsteerPath)
            return false;

        if (outPoints && outPointCount)
        {
            *outPointCount = nsteerPath;
            for (int i = 0; i < nsteerPath; ++i)
                dtVcopy(&outPoints[i * 3], &steerPath[i * 3]);
        }

        // Find vertex far enough to steer to.
        int ns = 0;
        while (ns < nsteerPath)
        {
            // Stop at Off-Mesh link or when point is further than slop away.
            if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
                !dtInRange(&steerPath[ns * 3], startPos, minTargetDist, 1000.0f))
                break;
            ns++;
        }
        // Failed to find good point to steer to.
        if (ns >= nsteerPath)
            return false;

        dtVcopy(steerPos, &steerPath[ns * 3]);
        steerPos[1]  = startPos[1];
        steerPosFlag = steerPathFlags[ns];
        steerPosRef  = steerPathPolys[ns];

        return true;
    }

    bool dtInRange(const float* v1, const float* v2, const float r, const float h)
    {
        const float dx = v2[0] - v1[0];
        const float dy = v2[1] - v1[1];
        const float dz = v2[2] - v1[2];
        return (dx * dx + dz * dz) < r * r && fabsf(dy) < h;
    }

    int dtFixupCorridor(dtPolyRef* path, const int npath, const int maxPath, const dtPolyRef* visited, const int nvisited)
    {
        int furthestPath    = -1;
        int furthestVisited = -1;

        // Find furthest common polygon.
        for (int i = npath - 1; i >= 0; --i)
        {
            bool found = false;
            for (int j = nvisited - 1; j >= 0; --j)
            {
                if (path[i] == visited[j])
                {
                    furthestPath    = i;
                    furthestVisited = j;
                    found           = true;
                }
            }
            if (found)
                break;
        }

        // If no intersection found just return current path.
        if (furthestPath == -1 || furthestVisited == -1)
            return npath;

        // Concatenate paths.

        // Adjust beginning of the buffer to include the visited.
        const int req  = nvisited - furthestVisited;
        const int orig = rcMin(furthestPath + 1, npath);
        int       size = rcMax(0, npath - orig);
        if (req + size > maxPath)
            size = maxPath - req;
        if (size)
            memmove(path + req, path + orig, size * sizeof(dtPolyRef));

        // Store visited
        for (int i = 0; i < req; ++i)
            path[i] = visited[(nvisited - 1) - i];

        return req + size;
    }

    // This function checks if the path has a small U-turn, that is,
    // a polygon further in the path is adjacent to the first polygon
    // in the path. If that happens, a shortcut is taken.
    // This can happen if the target (T) location is at tile boundary,
    // and we're (S) approaching it parallel to the tile edge.
    // The choice at the vertex can be arbitrary,
    //  +---+---+
    //  |:::|:::|
    //  +-S-+-T-+
    //  |:::|   | <-- the step can end up in here, resulting U-turn path.
    //  +---+---+
    int dtFixupShortcuts(dtPolyRef* path, int npath, dtNavMeshQuery* navQuery)
    {
        if (npath < 3)
            return npath;

        // Get connected polygons
        static const int maxNeis = 16;
        dtPolyRef        neis[maxNeis];
        int              nneis = 0;

        const dtMeshTile* tile = 0;
        const dtPoly*     poly = 0;
        if (dtStatusFailed(navQuery->getAttachedNavMesh()->getTileAndPolyByRef(path[0], &tile, &poly)))
            return npath;

        for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
        {
            const dtLink* link = &tile->links[k];
            if (link->ref != 0)
            {
                if (nneis < maxNeis)
                    neis[nneis++] = link->ref;
            }
        }

        // If any of the neighbour polygons is within the next few polygons
        // in the path, short cut to that polygon directly.
        static const int maxLookAhead = 6;
        int              cut          = 0;
        for (int i = dtMin(maxLookAhead, npath) - 1; i > 1 && cut == 0; i--)
        {
            for (int j = 0; j < nneis; j++)
            {
                if (path[i] == neis[j])
                {
                    cut = i;
                    break;
                }
            }
        }
        if (cut > 1)
        {
            int offset = cut - 1;
            npath -= offset;
            for (int i = 1; i < npath; i++)
                path[i] = path[i + offset];
        }

        return npath;
    }
} // namespace Navigation

CNavigationContainer::CNavigationContainer(uint16 zoneID)
: m_agentRadius(18.0f)
, m_navMesh(nullptr)
, m_query()
, m_filter()
, m_pathFindStatus(DT_FAILURE)
, m_startRef(0)
, m_endRef(0)
, m_polys()
, m_straightPathPolys()
, m_pickExtents()
, m_nstraightPath(0)
, m_straightPath()
, m_straightPathFlags()
, m_smoothPath()
, m_spos()
, m_epos()
, m_hitPos()
, m_hitNormal()
{
    m_filter.setIncludeFlags(POLYFLAGS_ALL ^ POLYFLAGS_DISABLED);
    m_filter.setExcludeFlags(0);
    m_pickExtents[0] = 2;
    m_pickExtents[1] = 4;
    m_pickExtents[2] = 2;

    for (auto const& [area, cost] : m_areaCosts)
    {
        m_filter.setAreaCost(area, cost);
    }
}

CNavigationContainer::~CNavigationContainer()
{
    if (m_navMesh != nullptr)
        dtFreeNavMesh(m_navMesh);
}

void CNavigationContainer::resetFilter()
{
    m_filter.setIncludeFlags(POLYFLAGS_ALL ^ POLYFLAGS_DISABLED);
    m_filter.setExcludeFlags(0);
}

void CNavigationContainer::applyFilter(unsigned short includePolyFlags, unsigned short excludePolyFlags)
{
    resetFilter();

    if (includePolyFlags != 0)
        m_filter.setIncludeFlags(includePolyFlags);
    if (excludePolyFlags != 0)
        m_filter.setExcludeFlags(excludePolyFlags);
}

float CNavigationContainer::getAgentRadius()
{
    return m_agentRadius;
}

float CNavigationContainer::getAreaCost(NavPolyAreas t_area)
{
    return m_areaCosts[t_area];
}

void CNavigationContainer::setAgentRadius(float t_radius)
{
    m_agentRadius = t_radius;
}

void CNavigationContainer::setAreaCost(NavPolyAreas t_area, float t_cost)
{
    m_areaCosts[t_area] = t_cost;
    m_filter.setAreaCost(t_area, t_cost);
}

bool CNavigationContainer::isMeshLoaded()
{
    return (m_navMesh != nullptr);
}

bool CNavigationContainer::loadFromFile(std::string const& t_filename)
{
    TracyZoneScoped;
    m_meshFilename = t_filename;
    std::ifstream file(m_meshFilename.c_str(), std::ios_base::in | std::ios_base::binary);

    if (!file.good())
    {
        return false;
    }

    NavMeshSetHeader header;
    file.read(reinterpret_cast<char*>(&header), sizeof(header));

    if (header.magic != Navigation::NAVMESHSET_MAGIC)
    {
        return false;
    }

    if (header.version != Navigation::NAVMESHSET_VERSION)
    {
        return false;
    }

    m_navMesh = dtAllocNavMesh();

    if (m_navMesh == nullptr)
    {
        ShowError("[NavigationContainer] Failed to allocate new dtNavMesh") return false;
    }

    dtStatus status = m_navMesh->init(&header.params);
    if (dtStatusFailed(status))
    {
        ShowError("[NavigationContainer] Failed to read dtNavMesh header.") return false;
    }

    for (int i = 0; i < header.numTiles; ++i)
    {
        NavMeshTileHeader tileHeader;
        file.read(reinterpret_cast<char*>(&tileHeader), sizeof(tileHeader));
        if (!tileHeader.tileRef || !tileHeader.dataSize)
        {
            break;
        }

        unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
        if (!data)
        {
            break;
        }
        memset(data, 0, tileHeader.dataSize);
        file.read(reinterpret_cast<char*>(data), tileHeader.dataSize);

        m_navMesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, nullptr);
    }

    // init detour nav mesh path finder
    status = m_query.init(m_navMesh, Navigation::MAX_POLYS);
    if (dtStatusFailed(status))
    {
        DebugNavmesh("[NavigationContainer] Failed to initialize dtNavMeshQuery") return false;
    }

    return true;
}

void CNavigationContainer::reloadMesh()
{
    if (isMeshLoaded())
        dtFreeNavMesh(m_navMesh);
    loadFromFile(m_meshFilename);
}

bool CNavigationContainer::isInWater(const position_t& t_pos)
{
    TracyZoneScoped;

    applyFilter(POLYFLAGS_SWIM, POLYFLAGS_ALL ^ POLYFLAGS_SWIM);

    Navigation::gameToDetourPosition(&t_pos, m_spos);

    float snearest[3];
    auto  status = m_query.findNearestPoly(m_spos, Navigation::smallPolyPickExt, &m_filter, &m_startRef, snearest);
    if (dtStatusFailed(status))
    {
        // Unable to find a poly, not in water
        return false;
    }

    // Found a poly, we're in water
    return true;
}

bool CNavigationContainer::isValidPosition(const position_t& t_pos)
{
    TracyZoneScoped;

    if (!isMeshLoaded())
        return false;

    float snearest[3];
    Navigation::gameToDetourPosition(&t_pos, m_spos);

    auto status = m_query.findNearestPoly(m_spos, Navigation::smallPolyPickExt, &m_filter, &m_startRef, snearest);
    if (dtStatusFailed(status))
    {
        ShowError("[NavigationContainer] isValidPosition failed to find starting position on dtNavMesh.");
        return false;
    }

    return m_navMesh->isValidPolyRef(m_startRef);
}

bool CNavigationContainer::isInRange(const position_t& t_start, const position_t& t_end, float t_distance, float t_h)
{
    TracyZoneScoped;

    Navigation::gameToDetourPosition(&t_start, m_spos);
    Navigation::gameToDetourPosition(&t_end, m_epos);

    return Navigation::dtInRange(m_spos, m_epos, t_distance, t_h);
}

bool CNavigationContainer::raycast(const position_t& t_start, const position_t& t_end, position_t& hit)
{
    TracyZoneScoped;

    if (!isMeshLoaded())
        return false;

    Navigation::gameToDetourPosition(&t_start, m_spos);
    Navigation::gameToDetourPosition(&t_end, m_epos);

    dtStatus polyStatus;
    polyStatus = m_query.findNearestPoly(m_spos, m_pickExtents, &m_filter, &m_startRef, 0);
    if (dtStatusFailed(polyStatus))
    {
        ShowError(fmt::format("[NavigationContainer] Failed to find starting position on dtNavMesh ({}, {}, {})", t_start.x, t_start.y, t_start.z));
        return false;
    }

    polyStatus = m_query.findNearestPoly(m_epos, m_pickExtents, &m_filter, &m_endRef, 0);
    if (dtStatusFailed(polyStatus))
    {
        ShowError(fmt::format("[NavigationContainer] Failed to find end position on dtNavMesh ({}, {}, {})", t_end.x, t_end.y, t_end.z));
        return false;
    }

    if (!m_navMesh->isValidPolyRef(m_startRef) || !m_navMesh->isValidPolyRef(m_endRef))
    {
        ShowError("[NavigationContainer] startRef or endRef are not valid poly references");
        return false;
    }

    m_nstraightPath   = 0;
    float t           = 0;
    m_npolys          = 0;
    m_nstraightPath   = 2;
    m_straightPath[0] = m_spos[0];
    m_straightPath[1] = m_spos[1];
    m_straightPath[2] = m_spos[2];

    auto status = m_query.raycast(m_startRef, m_spos, m_epos, &m_filter, &t, m_hitNormal, m_polys, &m_npolys, Navigation::MAX_POLYS);
    if (dtStatusFailed(status))
    {
        ShowError("[NavigationContainer] raycast failed raycast call");
        Navigation::detourError(status);
        return false;
    }

    if (t > 1)
    {
        dtVcopy(m_hitPos, m_epos);
        m_hitResult = false;
    }
    else
    {
        dtVlerp(m_hitPos, m_spos, m_epos, t);
        m_hitResult = true;
    }

    if (m_npolys > 0)
    {
        float h = 0;
        m_query.getPolyHeight(m_polys[m_npolys - 1], m_hitPos, &h);
        m_hitPos[1] = h;
    }

    Navigation::detourToGamePosition(hit, m_hitPos);
    return m_hitResult;
}

std::vector<pathpoint_t> CNavigationContainer::findStraightPath(const position_t& t_start, const position_t& t_end,
                                                                unsigned short includePolyFlags, unsigned short excludePolyFlags)
{
    TracyZoneScoped;

    std::vector<pathpoint_t> m_points;

    if (!isMeshLoaded())
        return m_points;

    applyFilter(includePolyFlags, excludePolyFlags);

    Navigation::gameToDetourPosition(&t_start, m_spos);
    Navigation::gameToDetourPosition(&t_end, m_epos);

    dtStatus polyStatus;
    polyStatus = m_query.findNearestPoly(m_spos, m_pickExtents, &m_filter, &m_startRef, 0);
    if (dtStatusFailed(polyStatus))
    {
        ShowError(fmt::format("[NavigationContainer] Failed to find starting position on dtNavMesh ({}, {}, {})", t_start.x, t_start.y, t_start.z));
        return m_points;
    }

    polyStatus = m_query.findNearestPoly(m_epos, m_pickExtents, &m_filter, &m_endRef, 0);
    if (dtStatusFailed(polyStatus))
    {
        ShowError(fmt::format("[NavigationContainer] Failed to find end position on dtNavMesh ({}, {}, {})", t_end.x, t_end.y, t_end.z));
        return m_points;
    }

    if (!m_navMesh->isValidPolyRef(m_startRef) || !m_navMesh->isValidPolyRef(m_endRef))
    {
        ShowError("[NavigationContainer] startRef or endRef are not valid poly references");
        return m_points;
    }

    m_pathFindStatus = DT_FAILURE;

    m_query.findPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter, m_polys, &m_npolys, Navigation::MAX_POLYS);
    m_nstraightPath = 0;
    if (m_npolys)
    {
        float epos[3];
        dtVcopy(epos, m_epos);
        if (m_polys[m_npolys - 1] != m_endRef)
        {
            m_query.closestPointOnPoly(m_polys[m_npolys - 1], m_epos, epos, 0);
        }

        m_query.findStraightPath(m_spos, epos, m_polys, m_npolys,
                                 m_straightPath, m_straightPathFlags, m_straightPathPolys, &m_nstraightPath,
                                 Navigation::MAX_POLYS, DT_STRAIGHTPATH_ALL_CROSSINGS);
    }
    else
    {
        m_npolys        = 0;
        m_nstraightPath = 0;
    }

    if (m_nstraightPath > 0)
    {
        for (int i = 3; i < m_nstraightPath * 3;)
        {
            float pathPos[3];
            pathPos[0] = m_straightPath[i++];
            pathPos[1] = m_straightPath[i++];
            pathPos[2] = m_straightPath[i++];
            Navigation::detourToGamePosition(pathPos);
            m_points.push_back({ { pathPos[0], pathPos[1], pathPos[2], 0, 0 }, 0, 0 });
        }
    }

    return m_points;
}

std::vector<pathpoint_t> CNavigationContainer::findSmoothPath(const position_t& t_start, const position_t& t_end,
                                                              unsigned short includePolyFlags, unsigned short excludePolyFlags)
{
    TracyZoneScoped;

    std::vector<pathpoint_t> m_points;

    if (m_navMesh == nullptr)
        return m_points;

    applyFilter(includePolyFlags, excludePolyFlags);

    // Convert positions to detour positions
    Navigation::gameToDetourPosition(&t_start, m_spos);
    Navigation::gameToDetourPosition(&t_end, m_epos);

    dtStatus polyStatus;
    polyStatus = m_query.findNearestPoly(m_spos, m_pickExtents, &m_filter, &m_startRef, 0);
    if (dtStatusFailed(polyStatus))
    {
        ShowError(fmt::format("[NavigationContainer] Failed to find starting position on dtNavMesh ({}, {}, {})", t_start.x, t_start.y, t_start.z));
        return m_points;
    }

    polyStatus = m_query.findNearestPoly(m_epos, m_pickExtents, &m_filter, &m_endRef, 0);
    if (dtStatusFailed(polyStatus))
    {
        ShowError(fmt::format("[NavigationContainer] Failed to find end position on dtNavMesh ({}, {}, {})", t_end.x, t_end.y, t_end.z));
        return m_points;
    }

    if (!m_navMesh->isValidPolyRef(m_startRef) || !m_navMesh->isValidPolyRef(m_endRef))
    {
        ShowError("[NavigationContainer] startRef or endRef are not valid poly references");
        return m_points;
    }

    m_pathFindStatus = DT_FAILURE;
    m_pathIterNum    = 0;

    if (m_startRef && m_endRef)
    {
        m_query.findPath(m_startRef, m_endRef, m_spos, m_epos, &m_filter, m_polys, &m_npolys, Navigation::MAX_POLYS);
        m_nsmoothpath = 0;

        if (m_npolys)
        {
            dtPolyRef polys[Navigation::MAX_POLYS];
            memcpy(polys, m_polys, sizeof(dtPolyRef) * m_npolys);
            int npolys = m_npolys;

            float iterPos[3], targetPos[3];
            m_query.closestPointOnPoly(m_startRef, m_spos, iterPos, 0);
            m_query.closestPointOnPoly(polys[npolys - 1], m_epos, targetPos, 0);

            m_nsmoothpath = 0;

            dtVcopy(&m_smoothPath[m_nsmoothpath * 3], iterPos);
            m_nsmoothpath++;

            while (npolys && m_nsmoothpath < Navigation::MAX_SMOOTH)
            {
                float         steerPos[3];
                unsigned char steerPosFlag;
                dtPolyRef     steerPosRef;

                if (!Navigation::dtGetSteerTarget(&m_query, iterPos, targetPos, Navigation::PATH_SLOP, polys, npolys, steerPos, steerPosFlag, steerPosRef))
                    break;

                bool endOfPath         = (steerPosFlag & DT_STRAIGHTPATH_END) ? true : false;
                bool offMeshConnection = (steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;

                float delta[3], len;
                dtVsub(delta, steerPos, iterPos);
                len = dtMathSqrtf(dtVdot(delta, delta));
                if ((endOfPath || offMeshConnection) && len < Navigation::PATH_STEP_SIZE)
                    len = 1;
                else
                    len = Navigation::PATH_STEP_SIZE / len;

                float moveTgt[3];
                dtVmad(moveTgt, iterPos, delta, len);

                // Move
                float     result[3];
                dtPolyRef visited[16];
                int       nVisited = 0;
                m_query.moveAlongSurface(polys[0], iterPos, moveTgt, &m_filter, result, visited, &nVisited, 16);
                npolys = Navigation::dtFixupCorridor(polys, npolys, Navigation::MAX_POLYS, visited, nVisited);
                npolys = Navigation::dtFixupShortcuts(polys, npolys, &m_query);

                float h = 0;
                m_query.getPolyHeight(polys[0], result, &h);
                result[1] = h;
                dtVcopy(iterPos, result);

                if (endOfPath && Navigation::dtInRange(iterPos, steerPos, Navigation::PATH_SLOP, 1.0f))
                {
                    dtVcopy(iterPos, targetPos);
                    if (m_nsmoothpath < Navigation::MAX_SMOOTH)
                    {
                        dtVcopy(&m_smoothPath[m_nsmoothpath * 3], iterPos);
                        m_nsmoothpath++;
                    }
                    break;
                }
                else if (offMeshConnection && Navigation::dtInRange(iterPos, steerPos, Navigation::PATH_SLOP, 1.0f))
                {
                    float startPos[3], endPos[3];

                    dtPolyRef prefRef = 0, polyRef = polys[0];
                    int       npos = 0;
                    while (npos < npolys && polyRef != steerPosRef)
                    {
                        prefRef = polyRef;
                        polyRef = polys[npos];
                        npos++;
                    }
                    for (int i = npos; i < npolys; ++i)
                    {
                        polys[i - npos] = polys[i];
                    }
                    npolys -= npos;

                    dtStatus status = m_navMesh->getOffMeshConnectionPolyEndPoints(prefRef, polyRef, startPos, endPos);
                    if (dtStatusSucceed(status))
                    {
                        if (m_nsmoothpath < Navigation::MAX_SMOOTH)
                        {
                            dtVcopy(&m_smoothPath[m_nsmoothpath * 3], startPos);
                            m_nsmoothpath++;
                            if (m_nsmoothpath & 1)
                            {
                                dtVcopy(&m_smoothPath[m_nsmoothpath * 3], startPos);
                                m_nsmoothpath++;
                            }
                        }

                        dtVcopy(iterPos, endPos);
                        float eh = 0.0f;
                        m_query.getPolyHeight(polys[0], iterPos, &eh);
                        iterPos[1] = eh;
                    }
                }

                // Store results
                if (m_nsmoothpath < Navigation::MAX_SMOOTH)
                {
                    dtVcopy(&m_smoothPath[m_nsmoothpath * 3], iterPos);
                    m_nsmoothpath++;
                }
            }
        }
    }
    else
    {
        m_npolys      = 0;
        m_nsmoothpath = 0;
    }

    if (m_nsmoothpath > 0)
    {
        for (int i = 3; i < m_nsmoothpath * 3;)
        {
            float pathPos[3];
            pathPos[0] = m_smoothPath[i++];
            pathPos[1] = m_smoothPath[i++];
            pathPos[2] = m_smoothPath[i++];
            Navigation::detourToGamePosition(pathPos);
            m_points.push_back({ { pathPos[0], pathPos[1], pathPos[2], 0, 0 }, 0, 0 });
        }
    }

    return m_points;
}

pathpoint_t CNavigationContainer::findRandomPosition(const position_t& t_around, float t_radius,
                                                     unsigned short includePolyFlags, unsigned short excludePolyFlags)
{
    TracyZoneScoped;

    applyFilter(includePolyFlags, excludePolyFlags);

    Navigation::gameToDetourPosition(&t_around, m_spos);

    float randomPt[3];
    float snearest[3];

    dtPolyRef randomRef;

    auto status = m_query.findNearestPoly(m_spos, Navigation::polyPickExt, &m_filter, &m_startRef, snearest);
    if (dtStatusFailed(status))
    {
        ShowError(fmt::format("[NavigationContainer] findRandomPosition failed to find start point ({}, {}, {})", t_around.x, t_around.y, t_around.z));
        return {};
    }

    if (!m_navMesh->isValidPolyRef(m_startRef))
    {
        ShowError("[NavigationContainer] findRandomPosition startRef was invalid.");
        return {};
    }

    status = m_query.findRandomPointAroundCircle(
        m_startRef, m_spos, t_radius, &m_filter, []() -> float
        { return xirand::GetRandomNumber(1.0f); },
        &randomRef, randomPt);

    if (dtStatusFailed(status))
    {
        ShowError("[NavigationContainer] findRandomPath Error");
        Navigation::detourError(status);
        return {};
    }

    Navigation::detourToGamePosition(randomPt);

    return {
        { randomPt[0], randomPt[1], randomPt[2], 0, 0 }
    };
}

bool CNavigationContainer::findClosestValidPoint(const position_t& t_start, float* t_point)
{
    TracyZoneScoped;

    if (!isMeshLoaded())
        return false;

    Navigation::gameToDetourPosition(&t_start, m_spos);

    auto status = m_query.findNearestPoly(m_spos, Navigation::bigPolyPickExt, &m_filter, &m_startRef, t_point);
    if (dtStatusFailed(status))
    {
        ShowError("[NavigationContainer] findClosestValidPoint Unable to any valid polys");
        return false;
    }

    Navigation::detourToGamePosition(t_point);
    return true;
}

bool CNavigationContainer::findFurthestValidPoint(const position_t& t_start, const position_t& t_end, float* t_point)
{
    TracyZoneScoped;

    if (!isMeshLoaded())
        return false;

    Navigation::gameToDetourPosition(&t_start, m_spos);
    Navigation::gameToDetourPosition(&t_end, m_epos);

    float startPoint[3];

    auto status = m_query.findNearestPoly(m_spos, Navigation::bigPolyPickExt, &m_filter, &m_startRef, startPoint);
    if (dtStatusFailed(status))
    {
        ShowError("[NavigationContainer] findFurthestValidPoint failed to find poly");
        return false;
    }

    dtPolyRef visited[16];
    int       visitedCount = 0;

    status = m_query.moveAlongSurface(m_startRef, startPoint, m_epos, &m_filter, t_point, visited, &visitedCount, 16);
    if (dtStatusFailed(status))
    {
        ShowError("[NavigationContainer] findFurthestValidPoint failed to find furthest point");
        return false;
    }

    Navigation::detourToGamePosition(t_point);
    return true;
}

void CNavigationContainer::snapToMesh(position_t& t_pos, float t_targetY, bool t_force)
{
    TracyZoneScoped;

    if (!isMeshLoaded() || !t_targetY || (!t_force && abs(t_pos.y - t_targetY) < 0.1f))
    {
        return;
    }

    float snearest[3];
    Navigation::gameToDetourPosition(&t_pos, m_spos);

    auto status = m_query.findNearestPoly(m_spos, Navigation::polyPickExt, &m_filter, &m_startRef, snearest);
    if (dtStatusFailed(status))
    {
        ShowError(fmt::format("[NavigationContainer] snapToMesh failed to find valid poly on dtNavMesh for given position ({}, {}, {})", t_pos.x, t_pos.y, t_pos.z));
        Navigation::detourError(status);
        return;
    }

    if (m_navMesh->isValidPolyRef(m_startRef))
    {
        Navigation::detourToGamePosition(snearest);
        t_pos.x = snearest[0];
        t_pos.y = snearest[1];
        t_pos.z = snearest[2];
    }
}
