
public Plugin myinfo = 
{
	name = "Drone",
	author = "Zeisen",
	description = "",
	version = "1.0",
	url = "http://steamcommunity.com/profiles/76561198002384750"
}

#include <sourcemod>
#include <sdktools>
#include <sdkhooks>
#include <navmesh>

#define MAXENTITIES 2048

enum DroneState
{
    DRONESTATE_IDLE = 0,
    DRONESTATE_MOVING = 1,
    DRONESTATE_MAX
}

enum struct DroneProp
{
    DroneState state;
    ArrayList goalPath;
    float goalPos[3];
}
DroneProp g_droneProp[MAXENTITIES + 1];
int g_laserModelIdx;

public void OnPluginStart()
{
    RegAdminCmd("sm_drone_moveto", Cmd_DroneMoveTo, ADMFLAG_ROOT);
}

public void OnMapStart()
{
	g_laserModelIdx = PrecacheModel("materials/sprites/laserbeam.vmt");
}

public Action Cmd_DroneMoveTo(int client, int args)
{
    float pos[3];
    GetClientEyePosition(client, pos);
    
    int drone = -1;
    while ((drone = FindEntityByClassname(drone, "drone")) != -1) {
        Drone_MoveTo(drone, pos);
    }
    
    return Plugin_Handled;
}

public void OnEntityCreated(int entity, const char[] classname)
{
    if (!StrEqual(classname, "drone"))
        return;
    
    SDKHook(entity, SDKHook_ThinkPost, Drone_OnThinkPost);
}

public void OnEntityDestroyed(int entity)
{
    char classname[32];
    GetEntityClassname(entity, classname, sizeof(classname));
    if (!StrEqual(classname, "drone"))
        return;
    
    delete g_droneProp[entity].goalPath;
}

public void Drone_OnThinkPost(int drone)
{
    if (g_droneProp[drone].goalPath != null) { 
        Drone_UpdatePathMovement(drone);
    }
}

void Drone_GetOrigin(int drone, float pos[3])
{
    GetEntPropVector(drone, Prop_Send, "m_vecOrigin", pos);
}

void Drone_UpdatePathMovement(int drone)
{
    if (g_droneProp[drone].goalPath == null)
        return;
    
    if (g_droneProp[drone].goalPath.Length == 0) {
        Drone_MoveTowardsPosition(drone, g_droneProp[drone].goalPos);
        return;
    }
    
    float dronePos[3];
    Drone_GetOrigin(drone, dronePos);
    
    float goalPos[3];
    g_droneProp[drone].goalPath.GetArray(0, goalPos, 3);
    
    // TE_SetupBeamPoints(dronePos,
        // goalPos,
        // g_laserModelIdx,
        // g_laserModelIdx,
        // 0,
        // 30,
        // 5.0,
        // 5.0,
        // 5.0,
        // 5, 
        // 0.0,
        // {255, 0, 0, 255},
        // 30);
        
    // TE_SendToAll();
    
    float distance = GetVectorDistance(dronePos, goalPos);
    if (distance <= 50.0) {
        g_droneProp[drone].goalPath.Erase(0);
        return;
    }
    
    Drone_MoveTowardsPosition(drone, goalPos);
}

void Drone_MoveTowardsPosition(int drone, const float pos[3])
{
    float dronePos[3];
    Drone_GetOrigin(drone, dronePos);

    float vecDroneToGoal[3];
    MakeVectorFromPoints(dronePos, pos, vecDroneToGoal);

    float vecDroneVel[3];
    NormalizeVector(vecDroneToGoal, vecDroneVel);
    ScaleVector(vecDroneVel, 200.0);

    TeleportEntity(drone, NULL_VECTOR, NULL_VECTOR, vecDroneVel);
}

public bool TraceRayWorld(int entity, int mask)
{
	return entity == 0;
}

public bool TraceRayDontHitEntity(int entity, int mask, any data)
{
	if (entity == data) return false;
	return true;
}

float GetMaxZ(const float pos[3])
{
    float result[3];

    Handle hTrace = TR_TraceRayFilterEx(pos,
        view_as<float>({-89.0, 0.0, 0.0}),
        MASK_PLAYERSOLID_BRUSHONLY,
        RayType_Infinite,
        TraceRayWorld);

    TR_GetEndPosition(result, hTrace);
    CloseHandle(hTrace);

    return result[2];
}

void Drone_MoveTo(int drone, const float goalPos[3])
{
    delete g_droneProp[drone].goalPath;

    float dronePos[3];
    Drone_GetOrigin(drone, dronePos);

    CNavArea startArea = NavMesh_GetNearestArea(dronePos);
    CNavArea goalID = NavMesh_GetNearestArea(goalPos);

    CNavArea closestArea = INVALID_NAV_AREA;

    NavMesh_BuildPath(startArea, 
        goalID,
        goalPos,
        NavMeshShortestPathCost,
        _,
        closestArea);

    CNavArea tempArea = closestArea;
    CNavArea parentArea = tempArea.Parent;

    int iNavDirection;
    float flHalfWidth;
    float flCenterPortal[3], flClosestPoint[3];
    
    ArrayList positions = new ArrayList(3);
    positions.PushArray(goalPos, 3);

    while (parentArea != INVALID_NAV_AREA)
    {
        float flTempAreaCenter[3], flParentAreaCenter[3];
        tempArea.GetCenter(flTempAreaCenter);
        parentArea.GetCenter(flParentAreaCenter);

        iNavDirection = tempArea.ComputeDirection(flParentAreaCenter);
        tempArea.ComputePortal(parentArea, iNavDirection, flCenterPortal, flHalfWidth);
        tempArea.ComputeClosestPointInPortal(parentArea, iNavDirection, flCenterPortal, flClosestPoint);

        flClosestPoint[2] = (tempArea.GetZ(flClosestPoint) + GetMaxZ(flClosestPoint)) / 2.0;

        positions.PushArray(flClosestPoint, 3);

        tempArea = parentArea;
        parentArea = tempArea.Parent;
    }
    
    startArea.GetCenter(dronePos);
    positions.PushArray(dronePos, 3);
    
    g_droneProp[drone].goalPath = new ArrayList(ByteCountToCells(12));
    for (int i = positions.Length - 1; i > 0; i--) {
        float pos[3];
        positions.GetArray(i, pos, 3);
        g_droneProp[drone].goalPath.PushArray(pos, 3);
    }
    g_droneProp[drone].goalPath.PushArray(goalPos, 3);
    g_droneProp[drone].goalPos = goalPos;
}