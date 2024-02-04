using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityStandardAssets.Vehicles.Car.Map;
using System.Threading;
using System.Security.Cryptography.X509Certificates;
//global variable path


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        private MapManager mapManager;
        private BoxCollider carCollider;

        private List<Vector3> path;
        private List<Vector3> obstacleMap;
        private Vector3 previousPosition;
        private float timeSinceLastProgress;
        private const float progressCheckInterval = 1.0f; // Time interval in seconds to check for progress
        private const float reverseDuration = 10.0f; // Time duration in seconds to reverse
        private bool isReversing = false;
        private float reverseStartTime;

        private int scaleFactor = 1;

        private int carSizex = 2;

        private int carSizey = 3;

        private float k_p = 2.9f;

        private float k_d;
       


        //public GameObject my_target;

        public bool driveInCircle = false;
        public float circleRadius = 0.15f;
        public float circleSpeed = 0.5f;
        float alpha = 0.2f;
        public Vector3 circleCenter = Vector3.zero;


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
        private List<Vector3> FindPath(Vector2Int startWorldPos, Vector2Int targetWorldPos)
        {
            //MapManager oldMapManager = new MapManager();

            //var oldObstacleMap = oldMapManager.GetObstacleMap();

            //oldObstacleMap.grid.cellSize.Set(0.1f, 0.1f, 0.1f);
            //MapManager mapManager = new MapManager();
            var obstacleMap = mapManager.GetObstacleMap();

            Dictionary<Vector2Int, ObstacleMap.Traversability> mapData = UpdateMapData(obstacleMap.traversabilityPerCell, startWorldPos);

            startWorldPos += new Vector2Int(0, 15); // idk offsetting the startPos away from the goalpost
            // NOTES: We are creating new Nodes in "Neighbour" and we are using these new Node Objects in the open/closed sets. When we try to use equal we are doing object reference. 
            //        so even if Nodes have same griddPoint they will be treated as different Nodes whhich in turn makes use pick the same Node multiple times thus a infinite loop. 
            // Neighbour added into OpenSet can be a Node we have already traversed. The algorithm wrongly chooses the node, it does not take the node with the lowest hCost when multiple nodes have same fCost
            startWorldPos *= scaleFactor;
            targetWorldPos *= scaleFactor;

            Debug.Log(startWorldPos);
            Debug.Log(targetWorldPos)
            ;


            Node startNode = new Node(startWorldPos, startWorldPos, targetWorldPos);
            Node targetNode = new Node(targetWorldPos, startWorldPos, targetWorldPos);



            Debug.Log(startNode.hCost);
            Debug.Log(targetNode.hCost);

            if (targetNode == null || startNode == null)
            {
                Debug.Log("Start/Target = Null");
                return new List<Vector3>();
            }


            //List<Node> openSet = new List<Node>();
            //HashSet<Node> closedSet = new HashSet<Node>();

            Dictionary<Vector2Int, Node> openSet1 = new Dictionary<Vector2Int, Node>();
            HashSet<Vector2Int> closedSet1 = new HashSet<Vector2Int>();


            //Dictionary<Vector2Int, ObstacleMap.Traversability> mapData = obstacleMap.traversabilityPerCell;


            //Dictionary<Vector2Int, ObstacleMap.Traversability> mapData = obstacleMap.traversabilityPerCell;

            Dictionary<Vector2Int, List<GameObject>> gameObjectsData = obstacleMap.gameGameObjectsPerCell; //LÄS: använd obstacleMap för att kolla för partial Blocks. Dom innehåller information om
                                                                                                           // hur blocken är blockerade

            Debug.LogError(mapData.Count);
            //Debug.LogError(newMapData.Count);
            //openSet.Add(startNode);
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
                Debug.Log("Current Node in OpenSet: " + currentNode.worldPosition);
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
                    Debug.Log("Target Reached!");
                    Debug.Log("Target Reached: startPos: " + startNode.worldPosition);
                    Debug.Log("Target Reached: goalPos: " + targetNode.worldPosition);

                    return RetracePath(startNode, targetNode);
                }


                List<Node> neighbours = GetNeighbours(currentNode, startWorldPos, targetWorldPos, mapData, gameObjectsData);

                Debug.Log("GetNeighbours Done!");
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
                        Debug.Log(currentNode.worldPosition);
                        Debug.Log("Node Parent" + openSet1[neighbour.worldPosition].parent.worldPosition);
                    }
                }
                oldNode = currentNode;


                j = j + 1;
            }
            Debug.Log("Did not reach target (end of function)!" + " J: " + j);
            return new List<Vector3>();
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

            Debug.Log("LowestCostNode: " + lowestCostNode.worldPosition);
            return lowestCostNode;
        }

        private List<Vector3> RetracePath(Node startNode, Node endNode)
        {
            Debug.Log("reTrace Entered");


            ObstacleMap obstacleMap = mapManager.GetObstacleMap();

            List<Vector3> path = new List<Vector3>();
            Debug.Log("reTrace: goalPos: " + endNode.worldPosition);

            Node currentNode = endNode;
            Node firstNode = startNode;

            while (currentNode.worldPosition != firstNode.worldPosition)
            {
                Debug.Log("Retracing Path");
                Vector2Int gridPosition = currentNode.worldPosition;

                Vector3 worldPosition = obstacleMap.grid.CellToLocalInterpolated(new Vector3Int(gridPosition.x / scaleFactor, gridPosition.y / scaleFactor, 0));
                Debug.Log("Added Path: " + worldPosition);

                path.Add(worldPosition);
                if (currentNode.parent == null)
                {
                    Debug.LogError("Found a node with no parent before reaching the start node");
                    break;
                }
                Debug.Log("RETRACE PATH: Current Node Parent: " + currentNode.parent.worldPosition);

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

        private static float GetDistance(Vector2Int start, Vector2Int goal)
        {
            float D = 1f;//float D = 10; // 
            float D2 = Mathf.Sqrt(2f); //float D2 = 14; // Mathf.Sqrt(2f);

            int dx = Mathf.Abs(goal.x - start.x);
            int dy = Mathf.Abs(goal.y - start.y);

            return (D * (dx + dy) + (D2 - 2 * D) * Mathf.Min(dx, dy));
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
                    Debug.Log("Added " + neighbourPos + " with fCost/hCost : " + neighbourNode.fCost + "/" + neighbourNode.hCost + " as Neighbour to " + node.worldPosition);

                    neighbours.Add(neighbourNode);
                }
            }

            return neighbours;
        }


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

            for (int x = -15; x <= 15; x++)
            {            // Clear cells in front of the car at the start post
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

        private List<Vector3> PathSmoothing(List<Vector3> path)
        { // not working correctly
            if (path.Count <= 2)
            {
                return path;
            }

            List<Vector3> smoothPath = new List<Vector3>();

            Vector3 currentPoint = path[0];

            for (int i = 2; i < path.Count; i++)
            {
                Vector3 nextPoint = path[i];
                if (!IsLineOfSightClear(currentPoint, nextPoint))
                {
                    smoothPath.Add(path[i - 1]);
                    currentPoint = path[i - 1];
                }

            }

            smoothPath.Add(path[path.Count - 1]);

            return smoothPath;
        }


        bool IsLineOfSightClear(Vector3 start, Vector3 end)
        {
            Vector3 direction = end - start;
            RaycastHit hit;
            float maxRange = 50f;
            if (Physics.Raycast(start + transform.up, transform.TransformDirection(direction), out hit, maxRange))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(direction) * hit.distance;
                if (Vector3.Distance(closestObstacleInFront, end) < 2f)
                { // close neough
                    return true;
                }
                //   Debug.Log("Did Hit");
            }
            return false;
        }



        private void Start()
        {
            carCollider = gameObject.transform.Find("Colliders/ColliderBottom").gameObject.GetComponent<BoxCollider>();
            // get the car controller
            m_Car = GetComponent<CarController>();
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




            // If you need more details, feel free to check out the ObstacleMap class internals.


            // Replace the code below that makes a random path
            // ...

            Vector3 goal_pos = mapManager.localGoalPosition;

            //Debug.Log(start_pos);
            Vector3 start_pos = mapManager.localStartPosition;

            var grid_start_pos = obstacleMap.grid.LocalToCell(start_pos);
            var grid_goal_pos = obstacleMap.grid.LocalToCell(goal_pos);

            //Debug.Log(grid_start_pos);
            List<Vector3> my_path = new List<Vector3>();


            Vector2Int start_vector2Int = new Vector2Int(Mathf.FloorToInt(grid_start_pos.x), Mathf.FloorToInt(grid_start_pos.y));
            Vector2Int goal_vector2Int = new Vector2Int(Mathf.FloorToInt(grid_goal_pos.x), Mathf.FloorToInt(grid_goal_pos.y));

            Debug.Log(start_vector2Int);
            this.path = FindPath(start_vector2Int, goal_vector2Int);

            this.path = PathSmoothing(this.path);
            //Debug.Log(my_path);
            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
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
            circleCenter = mapManager.grid.WorldToLocal(transform.position);

        }


        private void FixedUpdate()
        {
            
            var obstacleMap = mapManager.GetObstacleMap();

            Vector3 start_pos = mapManager.localStartPosition;
            Vector3 currentGridPosition = mapManager.grid.WorldToLocal(transform.position);

            Vector3 currentt_pos = mapManager.grid.WorldToLocal(transform.position);

            var grid_start_pos = obstacleMap.grid.LocalToCell(currentt_pos);


            var exampleObstacle = mapManager.GetObstacleMap().obstacleObjects[0];

            var globalPosition = transform.position;

            bool overlapped = Physics.ComputePenetration(
                carCollider,
                globalPosition,
                transform.rotation, // Use global position 
                exampleObstacle.GetComponent<MeshCollider>(), // Can take any collider and "project" it using position and rotation vectors.
                exampleObstacle.transform.position,
                exampleObstacle.transform.rotation,
                out var direction,
                out var distance
            );
            // 'out's give shortest direction and distance to "uncollide" two objects.
            if (overlapped || distance > 0)
            {
                // Means collider inside another   
            }
            // For more details https:docs.unity3d.com/ScriptReference/Physics.ComputePenetration.html
            ///////////////////////////

            // This is how you access information about the terrain from a simulated laser range finder
            // It might be wise to use this for error recovery, but do most of the planning before the race clock starts
            RaycastHit hit;
            float maxRange = 50f;
            if (Physics.Raycast(globalPosition + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
            {
                Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
                Debug.DrawRay(globalPosition, closestObstacleInFront, Color.yellow);
                //   Debug.Log("Did Hit");
            }

            Debug.DrawLine(globalPosition, mapManager.GetGlobalStartPosition(), Color.cyan); // Draw in global space
            Debug.DrawLine(globalPosition, mapManager.GetGlobalGoalPosition(), Color.blue);

            // Check and print traversability of currect position
            Vector3 myLocalPosition = mapManager.grid.WorldToLocal(transform.position); // Position of car w.r.p map coordinate origin (not world global)
            //(obstacleMap.IsLocalPointTraversable(myLocalPosition));

            // Execute your path here
            // ...
            if (path == null) // no target, only circle option works
            {
                old_target_pos = transform.position;
            }
            else
            {
                old_target_pos = path[0];
            }

            

            // Initialize circle at starting position
            circleCenter = transform.position;



            FollowPath();
        }

        private void FollowPath()
        {
            k_d = 0.18f*k_p;
            Vector3 target_position;


            Vector3 myLocalPosition = mapManager.grid.WorldToLocal(transform.position);
            //y is 0
            myLocalPosition.y = 0;
            
          
            if (driveInCircle) // for the circle option
            {
                alpha += Time.deltaTime * (circleSpeed / circleRadius);
                target_position = circleCenter + circleRadius * new Vector3((float)Math.Sin(alpha), 0f, (float)Math.Cos(alpha));
                target_velocity = circleSpeed * new Vector3((float)Math.Cos(alpha), 0f, -(float)Math.Sin(alpha));
            }
            else // if target is a game object
            {
                target_position = path[1];
                target_velocity = (target_position - old_target_pos) / Time.fixedDeltaTime;
            }

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

            if (Vector3.Distance(myLocalPosition, target_position) < 0.35f)
            {
                path.RemoveAt(0);
                //this.path.RemoveAt(0);

            }
            if(my_rigidbody.velocity.sqrMagnitude ==0f)
            {
                path.RemoveAt(0);
                //this.path.RemoveAt(0);
               
            }

            // this is how you control the car
            Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            m_Car.Move(1*steering, acceleration, acceleration, 0f);
        }
    }
}
