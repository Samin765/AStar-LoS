using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityStandardAssets.Vehicles.Car.Map;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        private MapManager mapManager;
        private BoxCollider carCollider;

        private int scaleFactor = 1;
        private List<Vector3> my_path;

        //define a class for A* search
        public class Node
        {
            public Vector3 position;
            public Node parent;
            public float g;
            public float h;
            public float f;
            public Node(Vector3 pos, Node par, float g, float h)
            {
                position = pos;
                parent = par;
                this.g = g;
                this.h = h;
                f = g + h;
            }
        }


        public List<Vector3> AStar(Vector3 start, Vector3 goal, Dictionary<Vector2Int, ObstacleMap.Traversability> mapData)

        {
            
            start *=scaleFactor;
            goal *=scaleFactor;



            Debug.LogError(start);
            Debug.LogError(goal);

            List<Vector3> path = new List<Vector3>();
            List<Node> openList = new List<Node>();
            List<Node> closeList = new List<Node>();
            Node startNode = new Node(start, null, 0, 0);
            openList.Add(startNode);
            while (openList.Count > 0)
            {
                Node currentNode = openList[0];
                for (int i = 1; i < openList.Count; i++)
                {
                    if (openList[i].f < currentNode.f)
                    {
                        currentNode = openList[i];
                    }
                }
                openList.Remove(currentNode);
                closeList.Add(currentNode);
                if (Distance(currentNode.position, goal) < 0.2f)
                {
                    Node temp = currentNode;
                    while (temp != null)
                    {
                        path.Add(temp.position/scaleFactor);
                        temp = temp.parent;
                    }

                    path.Reverse();
                    return path;
                }
                List<Node> successors = new List<Node>();
                Vector3[] directions = { new Vector3((float)0.5, 0, 0), new Vector3((float)-0.5, 0, 0), 
                                         new Vector3(0, 0, (float)0.5), new Vector3(0, 0, (float)-0.5), 
                                         new Vector3((float)-0.5, 0,(float)-0.5),
                                         new Vector3((float)0.5, 0 , (float)0.5)
                                         //,new Vector3((float)-0.5, 0 , (float)0.5),
                                         //new Vector3((float)0.5, 0 , (float)-0.5)
                                         };

                foreach (Vector3 dir in directions)
                {
                    Vector3 pos = currentNode.position + dir;
                    Vector2Int gridPos = new Vector2Int((int)pos.x, (int)pos.z);
                    if (pos == goal)
                    {
                        Node theNode = new Node(pos, currentNode, currentNode.g + 1, Distance(pos, goal));
                        successors.Add(theNode);
                        break;
                    }
                    if (!mapData.ContainsKey(gridPos) || mapData[gridPos] == ObstacleMap.Traversability.Blocked)
                    {
                        int ttt = 0;
                        continue;
                    }

                    Node newNode = new Node(pos, currentNode, currentNode.g + 1, Distance(pos, goal));
                    successors.Add(newNode);

                }
                foreach (Node successor in successors)
                {
                    //maybe a mistake in juding whether the node is in the closeList,juding only with the position
                    int flag = 0;
                    foreach (Node node in closeList)
                    {
                        if (node.position == successor.position)
                        {
                            flag = 1;
                            continue;
                        }
                    }
                    foreach (Node node in openList)
                    {
                        if (node.position == successor.position)
                        {

                            if (successor.g < node.g)
                            {
                                node.g = successor.g;
                                node.parent = currentNode;
                                node.f = node.g + node.h;
                            }
                            flag = 1;
                            continue;
                        }
                    }
                    if (flag == 0)
                    {
                        openList.Add(successor);
                    }
                }
            }
            return path;
        }
        //define a function to calculate the distance between two points,mahattan distance
        public float Distance1(Vector3 pos1, Vector3 pos2)
        {
            return Math.Abs(pos1.x - pos2.x) + Math.Abs(pos1.z - pos2.z);
        }

        public float Distance2(Vector3 pos1, Vector3 pos2)
        {
            float dx = pos1.x - pos2.x;
            float dz = pos1.z - pos2.z;
            return Mathf.Sqrt(dx * dx + dz * dz);
        }

        public float Distance(Vector3 pos1, Vector3 pos2)
{
    float dx = Mathf.Abs(pos1.x - pos2.x);
    float dz = Mathf.Abs(pos1.z - pos2.z);

    // Check if the movement is diagonal
    bool isDiagonal = dx > 0 && dz > 0;

    if (isDiagonal)
    {
        // For diagonal movement, use Euclidean distance multiplied by sqrt(2)
        return Mathf.Sqrt(dx * dx + dz * dz) * Mathf.Sqrt(2);
    }
    else
    {
        // For orthogonal movement, use Euclidean distance
        return Mathf.Sqrt(dx * dx + dz * dz);
    }
}

        


        private Dictionary<Vector2Int, ObstacleMap.Traversability> increaseResolution(Dictionary<Vector2Int, ObstacleMap.Traversability> oldMapData)
        {
            Dictionary<Vector2Int, ObstacleMap.Traversability> newMapData = new Dictionary<Vector2Int, ObstacleMap.Traversability>();

            //int scaleFactor = 10;
            // iterating over all elements in oldMapdata
            foreach (KeyValuePair<Vector2Int, ObstacleMap.Traversability> entry in oldMapData){

                Vector2Int gridPos = entry.Key;
                ObstacleMap.Traversability traversability = entry.Value;

                if (traversability == ObstacleMap.Traversability.Blocked ){
                    for (int x= 0; x < scaleFactor; x+= 1){
                        for (int y = 0; y < scaleFactor; y+= 1 ){
                            Vector2Int newPoint = new Vector2Int(gridPos.x *scaleFactor +x , gridPos.y *scaleFactor + y);

                            newMapData[newPoint] = ObstacleMap.Traversability.Blocked;
                        }
                    }
                }

                else if (traversability == ObstacleMap.Traversability.Free ) {
                    for (int x= 0; x < scaleFactor; x+= 1){
                        for (int y = 0; y < scaleFactor; y+= 1 ){
                            Vector2Int newPoint = new Vector2Int(gridPos.x * scaleFactor + x, gridPos.y *scaleFactor+ y);

                            newMapData[newPoint] = ObstacleMap.Traversability.Free;
                        }
                    }
                }

                
                else if (traversability == ObstacleMap.Traversability.Partial ) {
                    for (int x= 0; x < scaleFactor; x+= 1){
                        for (int y = 0; y < scaleFactor; y+= 1 ){
                            Vector2Int newPoint = new Vector2Int(gridPos.x * scaleFactor + x, gridPos.y *scaleFactor+ y);
                            //if

                            newMapData[newPoint] = ObstacleMap.Traversability.Free;
                        }
                    }
                }

                else {
                    Debug.LogError("IncreaseRes: GridPos not Blocked/Partial or Free ");
                }

            }
            
            return newMapData;
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
            Dictionary<Vector2Int, ObstacleMap.Traversability> mapData = increaseResolution(obstacleMap.traversabilityPerCell);
            Dictionary<Vector2Int, List<GameObject>> gameObjectsData = obstacleMap.gameGameObjectsPerCell;
            // Easy way to find all position vectors is either "Keys" in above dictionary or:
            foreach (var posThreeDim in obstacleMap.mapBounds.allPositionsWithin)
            {
                Vector2Int gridPos = new Vector2Int(posThreeDim.x, posThreeDim.z);
            }
            // If you need more details, feel free to check out the ObstacleMap class internals.


            // Replace the code below that makes a random path
            // ...

            Vector3 start_pos = mapManager.localStartPosition;
            //make the start_pas has the same y as the goal_pos

            Vector3 goal_pos = mapManager.localGoalPosition;
            //start_pos.y = (float)0.2;
            //goal_pos.y = start_pos.y;


            //set the goal near the start
            //goal_pos.x = start_pos.x -2;
            //use A* to find the path
            //set the start and the goal are integers' coordinates
           
            //whther the start and the goal are in the map or the blocked area
            
            //use mapdata in A* to judge whether the point is in the map

            my_path = AStar(start_pos, goal_pos, mapData);
            Vector3 old_wp = start_pos;
            foreach (var wp in my_path)
            {
                Debug.DrawLine(mapManager.grid.LocalToWorld(old_wp), mapManager.grid.LocalToWorld(wp), Color.red, 1000f);
                old_wp = wp;
            }
        }


        private void FixedUpdate()
        {
            // How to calculate if a physics collider overlaps another.
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
            var obstacleMap = mapManager.GetObstacleMap();
            Debug.Log(obstacleMap.IsLocalPointTraversable(myLocalPosition));

            // Execute your path here
            // ...

            // this is how you control the car
            //m_Car.Move(1f, 1f, 1f, 0f);
            FollowPath();
        }
    

    private void FollowPath()
{
    if (my_path != null && my_path.Count > 0)
    {
        Vector3 nextPoint = my_path[0];
        Vector3 directionToNextPoint = (nextPoint - transform.position).normalized;

        float angleToNextPoint = Vector3.Angle(transform.forward, directionToNextPoint);

        // Check if the next point is behind the car
        bool isNextPointBehind = angleToNextPoint > 90f;
        
        float steeringAngle = 0f;
        float acceleration = 0f;
        float brake = 0f;

        if (isNextPointBehind)
        {
            // Reverse if the next point is behind
            steeringAngle = Vector3.SignedAngle(transform.forward, -directionToNextPoint, Vector3.up);
            steeringAngle = Mathf.Clamp(steeringAngle / 45.0f, -1f, 1f);
            brake = -1.0f; // Negative acceleration for reversing
        }
        else
        {
            // Regular path following if the next point is in front
            steeringAngle = Vector3.SignedAngle(transform.forward, directionToNextPoint, Vector3.up);
            steeringAngle = Mathf.Clamp(steeringAngle / 45.0f, -1f, 1f);

            float maxSpeedInTurn = 0.5f; // Adjust this value as needed
            acceleration = Mathf.Lerp(0.2f, maxSpeedInTurn, Mathf.Abs(steeringAngle));

            if (Mathf.Abs(steeringAngle) > 0.5f) 
            {
                brake = 0.8f; // Apply brake in sharp turns
            }
        }

        m_Car.Move(steeringAngle, acceleration, brake, 0f);

        // Remove the waypoint if it's close enough
        if (Vector3.Distance(transform.position, nextPoint) < 1.0f)
        {
            my_path.RemoveAt(0);
        }
    }
}}}
