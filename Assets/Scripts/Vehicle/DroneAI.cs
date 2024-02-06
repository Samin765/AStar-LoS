using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using System;
using UnityStandardAssets.Vehicles.Car.Map;
using System.Threading;
using System.Security.Cryptography.X509Certificates;
using UnityEditor.Experimental.GraphView;
using UnityEngine.SocialPlatforms;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{
    private DroneController m_Drone; // the controller we want to use
 

        private MapManager mapManager;
        private BoxCollider carCollider;
        //private Pathfinding pathfinding;

        private List<Vector3> path;
        private List<Vector3> smoothedPath;
        private List<Vector3> obstacleMap;

        private int scaleFactor = 1;

        private int carSizex = 2;

        private int carSizey = 3;

        private float k_p = 100f;

        private float k_d;

        private enum ControlState
        {
            Normal,
            Reversing
        }

        private ControlState currentState = ControlState.Normal;
        private float reverseStartTime;

        private float lastDistanceToTarget = float.MaxValue;
        private float checkStuckInterval = 3f; // Time in seconds to check if we're getting closer
        private float lastCheckTime = 0f;

        //public GameObject my_target;

        public bool driveInCircle = false;
        public float circleRadius = 0.15f;
        public float circleSpeed = 0.5f;
        float alpha = 0.2f;
   

        //AI Path Smoothing
        public int smoothFragments = 4;
        public int smoothLookAhead = 1;
        public int smoothSkip = 1;

        //Reverse on collide with obstacle
        public float waypointRadius = 2f; 
        public float reverseCarSpeedThreshold = 1.1f;
        public float reverseDelayTime = 1f;
        public float reverseDuration = 1f;
        public float reverseForce = 20f;

        bool reversing = false;
        float reverseTime;
        Coroutine reverseCR;

        public Vector3 target_velocity;
        Vector3 old_target_pos;


        Rigidbody my_rigidbody;

        public class Node
        {
            public Vector2Int worldPosition;
            public Node parent;
            public float gCost;
            public float hCost;
            public float fCost { get { return gCost + hCost; } }

            public Node(Vector2Int _worldPos, Vector2Int _startPos, Vector2Int _targetPos)
            {
                parent = null;
                worldPosition = _worldPos;
                gCost = GetDistance(_startPos, _worldPos);
                hCost = GetDistance(_worldPos, _targetPos);
            }

            public override bool Equals(object obj)
            {
                return obj is Node node && worldPosition.Equals(node.worldPosition);
            }

            public override int GetHashCode()
            {
                return worldPosition.GetHashCode();
            }
        }

        //*** A* Algorithm
        private List<Vector3> FindPath(Vector2Int startWorldPos, Vector2Int targetWorldPos)
        {

            var obstacleMap = mapManager.GetObstacleMap();

            Dictionary<Vector2Int, ObstacleMap.Traversability> mapData = UpdateMapData(obstacleMap.traversabilityPerCell, startWorldPos);

            startWorldPos += new Vector2Int(0, 15); // idk offsetting the startPos away from the goalpost
            // NOTES: We are creating new Nodes in "Neighbour" and we are using these new Node Objects in the open/closed sets. When we try to use equal we are doing object reference. 
            //        so even if Nodes have same griddPoint they will be treated as different Nodes whhich in turn makes use pick the same Node multiple times thus a infinite loop. 
            // Neighbour added into OpenSet can be a Node we have already traversed. The algorithm wrongly chooses the node, it does not take the node with the lowest hCost when multiple nodes have same fCost
            startWorldPos *= scaleFactor;
            targetWorldPos *= scaleFactor;

            Debug.LogError(startWorldPos);


            Node startNode = new Node(startWorldPos, startWorldPos, targetWorldPos);
            Node targetNode = new Node(targetWorldPos, startWorldPos, targetWorldPos);





            if (targetNode == null || startNode == null)
            {
                Debug.Log("Start/Target = Null");
                return new List<Vector3>();
            }


            //List<Node> openSet = new List<Node>();
            //HashSet<Node> closedSet = new HashSet<Node>();

            Dictionary<Vector2Int, Node> openSet1 = new Dictionary<Vector2Int, Node>();
            HashSet<Vector2Int> closedSet1 = new HashSet<Vector2Int>();




            Dictionary<Vector2Int, List<GameObject>> gameObjectsData = obstacleMap.gameGameObjectsPerCell;


            Debug.LogError("MapData Count: " + mapData.Count);
            openSet1.Add(startWorldPos, startNode);
            var j = 0;
            Node oldNode = startNode;
            while (openSet1.Count > 0)
            {
                Node currentNode = GetLowestCostNode(openSet1);


                foreach (var pair in openSet1)
                {
                    Node node = pair.Value;
                    if (node.fCost < currentNode.fCost || (node.fCost == currentNode.fCost && node.hCost < currentNode.hCost))
                    {
                        currentNode = node;

                    }

                }
                //Debug.Log("Current Node in OpenSet: " + currentNode.worldPosition);
                if (currentNode == null)
                {
                    Debug.Log("CurrentNode == null!");
                    return new List<Vector3>();
                }



                openSet1.Remove(currentNode.worldPosition);
                closedSet1.Add(currentNode.worldPosition);

                if (currentNode.worldPosition == targetWorldPos)
                {
                    targetNode.parent = oldNode;
                    //Debug.Log("Target Reached!");
                    //Debug.Log("Target Reached: startPos: " + startNode.worldPosition);
                    //Debug.Log("Target Reached: goalPos: " + targetNode.worldPosition);

                    return RetracePath(startNode, targetNode);
                }


                List<Node> neighbours = GetNeighbours(currentNode, startWorldPos, targetWorldPos, mapData, gameObjectsData);

                //Debug.Log("GetNeighbours Done!");
                foreach (Node neighbour in neighbours)
                {
                    ObstacleMap.Traversability traversability = mapData[neighbour.worldPosition];

                    if (closedSet1.Contains(neighbour.worldPosition) || traversability == ObstacleMap.Traversability.Blocked)
                    {
                        continue;
                    }

                    float newCostToNeighbour = currentNode.gCost + GetDistance(currentNode.worldPosition, neighbour.worldPosition);
                    if (newCostToNeighbour < neighbour.gCost || !openSet1.ContainsKey(neighbour.worldPosition))
                    {
                        neighbour.gCost = newCostToNeighbour;
                        neighbour.hCost = GetDistance(neighbour.worldPosition, targetNode.worldPosition);
                        neighbour.parent = currentNode;

                        if (!openSet1.ContainsKey(neighbour.worldPosition))
                            openSet1[neighbour.worldPosition] = neighbour;
                        //Debug.Log(currentNode.worldPosition);
                        //Debug.Log("Node Parent" + openSet1[neighbour.worldPosition].parent.worldPosition);
                    }
                }
                oldNode = currentNode;


                j = j + 1;
            }
            //Debug.Log("Did not reach target (end of function)!" + " J: " + j);
            return new List<Vector3>();
        }

        private List<Vector3> SmoothPath(List<Vector3> pathToSmooth)
        {
            if (pathToSmooth == null || pathToSmooth.Count <= 0) return new List<Vector3>();
            List<Vector3> smoothPath = new List<Vector3>();
            int skipIndex = 0;


            for (int i = 0; i < pathToSmooth.Count; i++)
            {
                if (skipIndex >= smoothSkip)
                {
                    skipIndex = 0;
                    continue;
                }

                skipIndex++;

                //if(i == 0)
                //{
                //    //Start vector
                //    smoothPath.Add(pathToSmooth[0]);
                //}
                if(i == pathToSmooth.Count - 1)
                {
                    //End Vector
                    smoothPath.Add(pathToSmooth[pathToSmooth.Count - 1]);
                }
                else
                {
                    //Middle vectors
                    for (int j = 1; j < smoothFragments; j++)
                    {
                        var currentLookAhead = smoothLookAhead;

                        //Keep trying path closer to the reference path
                        while ((i + currentLookAhead >= pathToSmooth.Count) || pathToSmooth[i + currentLookAhead] == null)
                        {
                            currentLookAhead--;

                            if (i + currentLookAhead <= 0) break;
                        }

                        Vector3 start = pathToSmooth[i];
                        Vector3 end = pathToSmooth[i + currentLookAhead];

                        //var centerPivot = (start + end) * 0.5f;

                        //centerPivot -= new Vector3(0, -smoothOffset);

                        //var startRelativeCenter = start - centerPivot;
                       // var endRelativeCenter = end - centerPivot;

                        Vector3 lerpedPos = Vector3.Slerp(start, end, (float)j / (float)smoothFragments);
                        //Vector3 lerpedPos = Vector3.Slerp(pathToSmooth[i], pathToSmooth[i + currentLookAhead], (float)j / (float)smoothFragments);

                        smoothPath.Add(lerpedPos);
                    }
                }
            }

            return smoothPath;
        }

        private Node GetLowestCostNode(Dictionary<Vector2Int, Node> openSet)
        {
            Node lowestCostNode = null;
            float lowestCost = float.MaxValue;
            foreach (var node in openSet.Values)
            {
                if (node.fCost < lowestCost)
                {
                    lowestCostNode = node;
                    lowestCost = node.fCost;
                }
            }

           // Debug.Log("LowestCostNode: " + lowestCostNode.worldPosition);
            return lowestCostNode;
        }

        private List<Vector3> RetracePath(Node startNode, Node endNode)
        {
            //Debug.Log("reTrace Entered");


            ObstacleMap obstacleMap = mapManager.GetObstacleMap();

            List<Vector3> path = new List<Vector3>();
            //Debug.Log("reTrace: goalPos: " + endNode.worldPosition);

            Node currentNode = endNode;
            Node firstNode = startNode;

            while (currentNode.worldPosition != firstNode.worldPosition)
            {
                //Debug.Log("Retracing Path");
                Vector2Int gridPosition = currentNode.worldPosition;

                Vector3 worldPosition = obstacleMap.grid.CellToLocalInterpolated(new Vector3Int(gridPosition.x / scaleFactor, gridPosition.y / scaleFactor, 0));
                //Debug.Log("Added Path: " + worldPosition);

                path.Add(worldPosition);
                if (currentNode.parent == null)
                {
                    //Debug.LogError("Found a node with no parent before reaching the start node");
                    break;
                }
                //Debug.Log("RETRACE PATH: Current Node Parent: " + currentNode.parent.worldPosition);

                currentNode = currentNode.parent;
            }
            path.Reverse();
            return path;
        }

        private static int GetDistance1(Vector2Int worldPoisitionA, Vector2Int worldPositionB)
        {
            int distX = Mathf.Abs(Mathf.FloorToInt(worldPoisitionA.x - worldPositionB.x));
            int distY = Mathf.Abs(Mathf.FloorToInt(worldPoisitionA.y - worldPositionB.y));
            //if (distX > distY)
            //    return 14 * distY + 10 * (distX - distY);
            //return distX +  distY ;
            return Mathf.Min(Mathf.Abs(worldPoisitionA.x - worldPositionB.x), Mathf.Abs(worldPoisitionA.y - worldPositionB.y));
            //return Math.Abs(worldPoisitionA.x - worldPositionB.x) + Math.Abs(worldPoisitionA.y - worldPositionB.y);


            //return Mathf.RoundToInt(Vector3.Distance(worldPoisitionA, worldPositionB));
        }
        private static int GetDistance2(Vector2Int positionA, Vector2Int positionB)
        {
            int dx = positionA.x - positionB.x;
            int dy = positionA.y - positionB.y;
            return Mathf.FloorToInt(Mathf.Sqrt(dx * dx + dy * dy));
        }

        private static float GetDistanceWorking(Vector2Int start, Vector2Int goal)
        {
            float D = 1f;//float D = 10; // 
            float D2 = Mathf.Sqrt(2f); //float D2 = 14; // Mathf.Sqrt(2f);

            int dx = Mathf.Abs(goal.x - start.x);
            int dy = Mathf.Abs(goal.y - start.y);

            return (D * (dx + dy) + (D2 - 2 * D) * Mathf.Min(dx, dy));
        }
        


        private static float GetDistance(Vector2Int start, Vector2Int goal)
        {
                    Vector3 scaleVector = new Vector3(4.822223f, 15f, 30.16f);

    // Base cost for moving one unit horizontally or vertically
    float D = 1f;
    // Diagonal movement cost, assuming it's sqrt(scaleX^2 + scaleY^2)
    float D2 = Mathf.Sqrt(scaleVector.x * scaleVector.x+ scaleVector.z * scaleVector.z);

    int dx = Mathf.Abs(goal.x - start.x);
    int dy = Mathf.Abs(goal.y - start.y);

    // Scale the distances by their respective axis scaling factors
    float scaledDx = dx * scaleVector.x;
    float scaledDy = dy * scaleVector.z;

    // Use the scaled distances to calculate the total distance
    // Note: The heuristic adjusts to account for the minimum scaled distance being used for diagonal movement
    return (D * (scaledDx + scaledDy) + (D2 - 2 * D) * Mathf.Min(scaledDx, scaledDy));
        }

        private List<Node> GetNeighbours(Node node, Vector2Int startPos, Vector2Int targetPos, Dictionary<Vector2Int, ObstacleMap.Traversability> newMapData, Dictionary<Vector2Int, List<GameObject>> gameObjectsData)
        {
            List<Node> neighbours = new List<Node>();
            var obstacleMap = mapManager.GetObstacleMap();
            Dictionary<Vector2Int, ObstacleMap.Traversability> mapData = newMapData;

            // Include diagonal directions
            Vector2Int[] directions = new Vector2Int[]
            {
                new Vector2Int(1, 0), new Vector2Int(-1, 0),
                new Vector2Int(0, 1), new Vector2Int(0, -1)
                ,new Vector2Int(1, 1), new Vector2Int(-1, -1)
                ,new Vector2Int(1, -1), new Vector2Int(-1, 1)
            };

            foreach (var dir in directions)
            {
                Vector2Int neighbourPos = node.worldPosition + dir;

                if (mapData.ContainsKey(neighbourPos) && (mapData[neighbourPos] == ObstacleMap.Traversability.Free || mapData[neighbourPos] == ObstacleMap.Traversability.Partial))
                {
                    Node neighbourNode = new Node(neighbourPos, startPos, targetPos);
                    //Debug.Log("Added " + neighbourPos + " with fCost/hCost : " + neighbourNode.fCost + "/" + neighbourNode.hCost + " as Neighbour to " + node.worldPosition);

                    neighbours.Add(neighbourNode);
                }
            }

            return neighbours;
        }

        // not used
        public bool IsPointInsideAnyBuilding(Vector2Int gridPoint, List<GameObject> buildings)
        {

            if (buildings == null)
            {
                return false;
            }
            ObstacleMap obstacleMap = mapManager.GetObstacleMap();

            Vector3 localPoint = obstacleMap.grid.CellToLocalInterpolated(new Vector3(gridPoint.x, 0, gridPoint.y));
            foreach (var building in buildings)
            {
                Vector3 pointInBuildingSpace = building.transform.InverseTransformPoint(localPoint);

                if (building.GetComponent<Collider>().bounds.Contains(pointInBuildingSpace))
                {
                    return true; // Point is inside this building
                }
            }

            return false; // Point is not inside any building
        }

        // Updates MapData to create a bigger boundary between blocked objects in refernce to carsize
        private Dictionary<Vector2Int, ObstacleMap.Traversability> UpdateMapData(Dictionary<Vector2Int, ObstacleMap.Traversability> mapData, Vector2Int startPos)
        {
            List<Vector2Int> cellsToUpdate = new List<Vector2Int>();
            var obstacleMap = mapManager.GetObstacleMap();
            //carSizex = Mathf.RoundToInt(0.005f/(obstacleMap.grid.cellSize.x - obstacleMap.grid.cellSize.y ));
            //carSizey = carSizex;

            foreach (var cell in mapData)
            {
                if (cell.Value == ObstacleMap.Traversability.Blocked)
                {
                    for (int x = -carSizex; x <= carSizex; x++)
                    {
                        for (int y = -carSizey; y <= carSizey; y++)
                        {

                            Vector2Int neighbourPos = new Vector2Int(cell.Key.x + x, cell.Key.y + y);
                            if (GetDistance(neighbourPos, startPos) < 2)
                            {

                                continue; // Skip updating this cell as it's near the starting position
                            }

                            else if (mapData.ContainsKey(neighbourPos) && mapData[neighbourPos] != ObstacleMap.Traversability.Blocked)
                            {
                                cellsToUpdate.Add(neighbourPos);
                            }
                        }
                    }
                }
            }

            foreach (var cellPos in cellsToUpdate)
            {
                mapData[cellPos] = ObstacleMap.Traversability.Blocked;
            }
            // Clear cells in front of the car at the start post
            for (int x = -15; x <= 15; x++)
            {
                for (int y = 0; y <= 9; y++)
                {
                    Vector2Int cellInFront = startPos + new Vector2Int(x, y);
                    if (mapData.ContainsKey(cellInFront))
                    {
                        mapData[cellInFront] = ObstacleMap.Traversability.Free;
                    }
                }
            }

            return mapData;
        }

        // not working
        bool IsLineOfSightClear(Vector3 start, Vector3 end)
        {
            Vector3 direction = end - start;
            direction.Normalize();
            RaycastHit hit;
            float maxRange = 50f;
            if (Physics.Raycast(start + transform.up, transform.TransformDirection(direction), out hit, maxRange))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(direction) * hit.distance;
                if (Vector3.Distance(start, end) < hit.distance)
                { // close neough
                    Debug.LogError("Hit in PAthSmoothing");
                    return true;
                }
                //   Debug.Log("Did Hit");
            }
            return false;
        }

        private void Awake()
        {
            //pathfinding = GetComponent<Pathfinding>();
        }

        private void Start()
        {
           //carCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();
            // get the car controller
            Debug.Log("gl");
            m_Drone = GetComponent<DroneController>();
            mapManager = FindObjectOfType<GameManager>().mapManager;


            // Plan your path here
            Vector3 someLocalPosition = mapManager.grid.WorldToLocal(transform.position); // Position of car w.r.p map coordinate origin (not world global)
            // transform.localRotation;  Rotation w.r.p map coordinate origin (not world global)

            // This is how you access information about specific points
            var obstacleMap = mapManager.GetObstacleMap();
            obstacleMap.IsLocalPointTraversable(someLocalPosition);

            // Local to grid . See other methods for more.
            obstacleMap.grid.LocalToCell(someLocalPosition);

            // This is how you access a traversability grid or gameObjects in each cell.
            Dictionary<Vector2Int, ObstacleMap.Traversability> mapData = obstacleMap.traversabilityPerCell;
            Dictionary<Vector2Int, List<GameObject>> gameObjectsData = obstacleMap.gameGameObjectsPerCell;
            // Easy way to find all position vectors is either "Keys" in above dictionary or:
            foreach (var posThreeDim in obstacleMap.mapBounds.allPositionsWithin)
            {
                Vector2Int gridPos = new Vector2Int(posThreeDim.x, posThreeDim.z);
            }


            Vector3 goal_pos = mapManager.localGoalPosition;

            //Debug.Log(start_pos);
            Vector3 start_pos = mapManager.localStartPosition;
            Debug.LogError("Local start: " +start_pos);
            Debug.LogError("World: " + transform.position);
            Debug.LogError("World/Transform start to Local: " + mapManager.grid.WorldToLocal(transform.position));
            var grid_start_pos = obstacleMap.grid.LocalToCell(start_pos);
            var grid_goal_pos = obstacleMap.grid.LocalToCell(goal_pos);

            //Debug.Log(grid_start_pos);
            List<Vector3> my_path = new List<Vector3>();


            Vector2Int start_vector2Int = new Vector2Int(Mathf.FloorToInt(grid_start_pos.x), Mathf.FloorToInt(grid_start_pos.y));
            Vector2Int goal_vector2Int = new Vector2Int(Mathf.FloorToInt(grid_goal_pos.x), Mathf.FloorToInt(grid_goal_pos.y));


            //this.path = FindPath(start_vector2Int, goal_vector2Int);
            List<Vector3> unSmoothPath = FindPath(start_vector2Int, goal_vector2Int);

            //Apply Path Smoothing
            this.path =unSmoothPath;

            DrawDebugLineOnPath();

            Vector3 old_wp = start_pos;
            foreach (var wp in this.path)
            {
                //Debug.Log(wp);
                //Debug.Log("---");
                Debug.DrawLine(mapManager.grid.LocalToWorld(old_wp), mapManager.grid.LocalToWorld(wp), Color.red, 1000f);
                old_wp = wp;
            }
            my_rigidbody = GetComponent<Rigidbody>();
            old_target_pos = path[0];
        }
        
        public void DrawDebugLineOnPath()
        {
            for (int i = 0; i < path.Count; i++)
            {
                if (i == 0)
                {
                    //Start vector
                    if(path [i + 1] != null)
                        Debug.DrawLine(path[i], path[i] - path[i + 1], Color.blue, 1000f);
                }
                else if (i == path.Count - 1)
                {
                    //End Vector
                    if (path[i - 1] != null)
                        Debug.DrawLine(path[i], path[i] - path[i - 1], Color.blue, 1000f);
                }
                else
                {
                    if (path[i + 1] != null)
                        Debug.DrawLine(path[i], path[i] - path[i + 1], Color.blue, 1000f);
                }
            }
        }


        private void FixedUpdate()
        {




            FollowPath();
        }

          private void FollowPath()
    {
        k_p = 40f;

         k_p = 1f;
        
        Vector3 target_position;


        Vector3 myLocalPosition = mapManager.grid.WorldToLocal(transform.position) ;
        //y is 0
        myLocalPosition.y = 0;
        target_position = this.path[1];
        target_velocity = (target_position - old_target_pos) / Time.fixedDeltaTime;
        old_target_pos = target_position;

        // a PD-controller to get desired acceleration from errors in position and velocity
        Vector3 position_error = target_position - myLocalPosition;
        Vector3 velocity_error = target_velocity - my_rigidbody.velocity;
        Vector3 desired_acceleration = k_p * position_error + k_d * velocity_error;

        float steering = Vector3.Dot(desired_acceleration, transform.right);
        float acceleration = Vector3.Dot(desired_acceleration, transform.forward);
        

        Debug.DrawLine(target_position, target_position + target_velocity, Color.red);
        Debug.DrawLine(myLocalPosition, myLocalPosition + my_rigidbody.velocity, Color.blue);
        Debug.DrawLine(myLocalPosition, myLocalPosition + desired_acceleration, Color.black);



        var acceleration_x = position_error.normalized.x;
        var acceleration_y = position_error.normalized.z;

        Vector3 norm_acceleration = desired_acceleration.normalized;
        // this is how you control the car
        //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
                    var dampingFactor_x = 0.12f;
                var dampingFactor_y = 0.12f;
        if(my_rigidbody.velocity.x > 8){
            dampingFactor_x = 0.2f;
            if(my_rigidbody.velocity.y > 8){
                dampingFactor_y = 0.2f;
            }
        }
        else{
            
        }
        


        float adjustedX = norm_acceleration.x - (dampingFactor_x * my_rigidbody.velocity.x);
        float adjustedZ = norm_acceleration.z - (dampingFactor_y * my_rigidbody.velocity.z);

        Vector3 directionToWaypoint = (target_position - transform.position).normalized;


        
                
    float maxRange = 50f;
    RaycastHit hit;
    Vector3 globalPosition = transform.position;

    // Check for obstacle to the right
    Vector3 rightVector = new Vector3(0.1f, 0f, 0f);
    var rotatedVectorRight = Quaternion.AngleAxis(-45, Vector3.up) * rightVector;

    Vector3 leftVector = new Vector3(-0.1f, 0f, 0f);
    var rotatedVectorLeft = Quaternion.AngleAxis(45, Vector3.up) * leftVector;
                     if (Physics.Raycast(globalPosition + transform.up, transform.TransformDirection(rotatedVectorRight), out hit, maxRange)) 
    {   
        //Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.right) * hit.distance;
        if (hit.distance < 10f) // 
        {

            Debug.LogError("Hit to the right");

            Vector3 closestObstacleInFront = transform.TransformDirection(rotatedVectorRight) * hit.distance;
            //Debug.DrawRay(globalPosition, closestObstacleInFront, Color.yellow);

            adjustedX = adjustedX - 50/hit.distance;

            
        }
    }


    // Check for obstacle to the left
    else if (Physics.Raycast(globalPosition + transform.up, transform.TransformDirection(rotatedVectorLeft), out hit, maxRange))
    {
        if (hit.distance < 10f && hit.distance >5f) 
        {


            Debug.LogError("Hit to the left");
            Vector3 closestObstacleInFront = transform.TransformDirection(rotatedVectorLeft) * hit.distance;
            //Debug.DrawRay(globalPosition, closestObstacleInFront, Color.yellow);

            
            adjustedX = adjustedX + 50/hit.distance;
            
            //steeringAdjustmentFactor = 0.5f; // Adjust this value as needed
        }
    }

  // Check for obstacle forward

    if (Physics.Raycast(globalPosition + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
    {
        if (hit.distance < 6f && 1.5f <hit.distance) 
        {


            Debug.LogError("Hit forward");
            Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
            //Debug.DrawRay(globalPosition, closestObstacleInFront, Color.yellow);


            adjustedZ = adjustedZ - 10f/hit.distance;

            


            //steeringAdjustmentFactor = 0.5f; // Adjust this value as needed
        }
        else if(hit.distance < 1.5f){
            adjustedZ = adjustedZ - 15;
        }
    }
        

           m_Drone.Move(adjustedX, adjustedZ);
        if (Vector3.Distance(myLocalPosition, target_position) < 0.35f)
        {
            
            this.path.RemoveAt(0);

        }

        
    

 IEnumerator Co_ResetReverAfterTime()
        {
            yield return new WaitForSeconds(reverseDuration);
            reversing = false;
        }
    }
}


        
    
