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
        
        private List<Vector3> path;
        private List<Vector3> obstacleMap;
        public class Node
        {
            public Vector2 worldPosition;
            public Node parent;
            public int gCost;
            public int hCost;
            public int fCost { get { return gCost + hCost; } }

            public Node(Vector2 _worldPos, Vector2 _targetPos, Vector2 _startPos)
            {
                
                worldPosition = _worldPos;
                gCost = GetDistance(_startPos, _worldPos );
                hCost = GetDistance(_worldPos, _targetPos);
            }

            public Node(Vector2 _worldPos)
            {
                
                worldPosition = _worldPos;
                
            }

            
        }
        private List<Vector3> FindPath(Vector2 startWorldPos, Vector2 targetWorldPos)
        {
            Node startNode = new Node(startWorldPos, startWorldPos, targetWorldPos);
            Node targetNode = new Node(targetWorldPos, startWorldPos, targetWorldPos);

            if (targetNode == null || startNode == null) {
                Debug.Log("Start/Target = Null");
                return new List<Vector3>();
            }


            List<Node> openSet = new List<Node>();
            HashSet<Node> closedSet = new HashSet<Node>();

            openSet.Add(startNode);
            var j = 0;
            while (openSet.Count > 0)
            {
                Node currentNode = openSet[0];

                if (currentNode == null){
                    Debug.Log("CurrentNode == null!");
                    return new List<Vector3>();
                }



                for (int i = 1; i < openSet.Count; i++)
                {
                    if (openSet[i].fCost < currentNode.fCost || (openSet[i].fCost == currentNode.fCost && openSet[i].hCost < currentNode.hCost))
                    {
                        currentNode = openSet[i];
                    }
                }

                openSet.Remove(currentNode);
                closedSet.Add(currentNode);

                if (currentNode.worldPosition == targetNode.worldPosition)
                {
                    Debug.Log("Target Reached!");
                    return RetracePath(startNode, targetNode);
                }

                foreach (Node neighbour in GetNeighbours(currentNode, startWorldPos, targetWorldPos))
                {
                    var obstacleMap = mapManager.GetObstacleMap();

                    if (closedSet.Contains(neighbour) || !(obstacleMap.IsLocalPointTraversable(neighbour.worldPosition) == ObstacleMap.Traversability.Free))
                    {
                        continue;
                    }

                    int newCostToNeighbour = currentNode.gCost + GetDistance(currentNode.worldPosition, neighbour.worldPosition);
                    if (newCostToNeighbour < neighbour.gCost || !(openSet.Contains(neighbour)))
                    {
                        neighbour.gCost = newCostToNeighbour;
                        neighbour.hCost = GetDistance(neighbour.worldPosition, targetNode.worldPosition);
                        neighbour.parent = currentNode;

                        if (!openSet.Contains(neighbour))
                            openSet.Add(neighbour);
                    }
                }

                j = j + 1;
            }
            Debug.Log("Did not reach target!");
            return new List<Vector3>();
        }

        private List<Vector3> RetracePath(Node startNode, Node endNode)
        {
            List<Vector3> path = new List<Vector3>();
            Node currentNode = endNode;

            while (currentNode != startNode)
            {
                path.Add(currentNode.worldPosition);
                currentNode = currentNode.parent;
            }
            path.Reverse();
            return path;
        }
        private static int GetDistance(Vector2 worldPoisitionA, Vector2 worldPositionB)
        {
            int distX = Mathf.Abs(Mathf.RoundToInt(worldPoisitionA.x - worldPositionB.x));
            int distY = Mathf.Abs(Mathf.RoundToInt(worldPoisitionA.y - worldPositionB.y));
            if (distX > distY)
                return 14 * distY + 10 * (distX - distY);
            return 14 * distX + 10 * (distY - distX);
            //return Mathf.RoundToInt(Vector3.Distance(worldPoisitionA, worldPositionB));
        }
        private List<Node> GetNeighbours(Node node, Vector2 startPos, Vector2 targetPos)
        {
            List<Node> neighbours = new List<Node>();
            var obstacleMap = mapManager.GetObstacleMap();
            Dictionary<Vector2Int, ObstacleMap.Traversability> mapData = obstacleMap.traversabilityPerCell;
            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    if (x == 0 && y == 0)
                        continue;


                    var worldPoint = node.worldPosition + new Vector2(x, y);
                    Vector2Int vector2Int = new Vector2Int(Mathf.RoundToInt(worldPoint.x), Mathf.RoundToInt(worldPoint.y));



                    if (mapData.ContainsKey(vector2Int))
                    {
                        ObstacleMap.Traversability traversability = mapData[vector2Int];

                        if (traversability == ObstacleMap.Traversability.Free)
                        {
                            Debug.Log("Added Neighbour");
                            neighbours.Add(new Node(worldPoint, startPos, targetPos ));
                        }



                    }


                }
            }

            return neighbours;
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
            var grid_goal_pos =  obstacleMap.grid.LocalToCell(goal_pos);
            
            Debug.Log(grid_start_pos);
            List<Vector3> my_path = new List<Vector3>();

            my_path = FindPath(start_pos, goal_pos);
            //Debug.Log(my_path);
            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            Vector3 old_wp = start_pos;
            foreach (var wp in my_path)
            {
                Debug.Log(wp);
                Debug.Log("---");
                Debug.DrawLine(mapManager.grid.LocalToWorld(old_wp), mapManager.grid.LocalToWorld(wp), Color.white, 1000f);
                old_wp = wp;
            }
        }


        private void FixedUpdate()
        {
            var obstacleMap = mapManager.GetObstacleMap();

            Vector3 start_pos = mapManager.localStartPosition;
            Vector3 currentGridPosition = mapManager.grid.WorldToLocal(transform.position);
            //Vector3 world_pos = mapManager.worldStartPosition;

            Vector3 currentt_pos = mapManager.grid.WorldToLocal(transform.position);

            var grid_start_pos = obstacleMap.grid.LocalToCell(currentt_pos);
            //Debug.Log(currentt_pos);
            // How to calculate if a physics collider overlaps another.
            //Debug.Log(start_pos);
            //Debug.Log(currentGridPosition);

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

            // this is how you control the car
            m_Car.Move(0f, 1f, 1f, 0f);
            //FollowPath();
        }

        private void FollowPath()
        {
            //Debug.Log(path)
            if (path != null && path.Count > 0)
            {
                Vector3 nextPoint = path[0];
                Vector3 directionToNextPoint = (nextPoint - transform.position).normalized;

                float steeringAngle = Vector3.SignedAngle(transform.forward, directionToNextPoint, Vector3.up);
                steeringAngle = Mathf.Clamp(steeringAngle / 45.0f, -1f, 1f);

                float acceleration = 1.0f;
                float brake = 0.0f;

                m_Car.Move(steeringAngle, acceleration, brake, 0f);

                if (Vector3.Distance(transform.position, nextPoint) < 1.0f)
                {
                    path.RemoveAt(0);
                }
            }
        }
    }
}
