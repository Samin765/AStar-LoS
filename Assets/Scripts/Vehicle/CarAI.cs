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
            public Vector2Int worldPosition;
            public Node parent;
            public int gCost;
            public int hCost;
            public int fCost { get { return gCost + hCost; } }

            public Node(Vector2Int _worldPos, Vector2Int _startPos, Vector2Int _targetPos)
            {
                
                worldPosition = _worldPos;
                gCost = GetDistance(_startPos, _worldPos );
                hCost = GetDistance(_worldPos, _targetPos);
            }


            
        }
        private List<Vector3> FindPath(Vector2Int startWorldPos, Vector2Int targetWorldPos)
        {
            // NOTES: We are creating new Nodes in "Neighbour" and we are using these new Node Objects in the open/closed sets. When we try to use equal we are doing object reference. 
            //        so even if Nodes have same worldPoint they will be treated as different Nodes whhich in turn makes use pick the same Node multiple times thus a infinite loop. 
            // Neighbour added into OpenSet can be a Node we have already traversed. The algorithm wrongly chooses the node, it does not take the node with the lowest hCost when multiple nodes have same fCost
            Debug.Log(startWorldPos);
            Debug.Log(targetWorldPos);
            Node startNode = new Node(startWorldPos, startWorldPos, targetWorldPos);
            Node targetNode = new Node(targetWorldPos, startWorldPos, targetWorldPos);
            Debug.Log(startNode.hCost);
            Debug.Log(targetNode.hCost);
            if (targetNode == null || startNode == null) {
                Debug.Log("Start/Target = Null");
                return new List<Vector3>();
            }

            
            List<Node> openSet = new List<Node>();
            HashSet<Node> closedSet = new HashSet<Node>();
            var obstacleMap = mapManager.GetObstacleMap();
            Dictionary<Vector2Int, ObstacleMap.Traversability> mapData = obstacleMap.traversabilityPerCell;
            openSet.Add(startNode);

            var j = 0;
            while (openSet.Count > 0 && j < 4)
            {
                Node currentNode = openSet[0];
                Debug.Log("Current Node in OpenSet: " + currentNode.worldPosition);
                if (currentNode == null){
                    Debug.Log("CurrentNode == null!");
                    return new List<Vector3>();
                }



                for (int i = 1; i < openSet.Count; i++)
                {
                    if ((openSet[i].fCost < currentNode.fCost || openSet[i].fCost == currentNode.fCost) && openSet[i].hCost < currentNode.hCost)
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


                List<Node> neighbours = GetNeighbours(currentNode, startWorldPos, targetWorldPos);

                Debug.Log("GetNeighbours Done!");
                foreach (Node neighbour in neighbours)
                {
                    ObstacleMap.Traversability traversability = mapData[neighbour.worldPosition];

                    if (closedSet.Contains(neighbour) || !(traversability == ObstacleMap.Traversability.Free))
                    {
                        continue;
                    }

                    int newCostToNeighbour = currentNode.gCost + GetDistance(currentNode.worldPosition, neighbour.worldPosition);
                    if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
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
            Debug.Log("Did not reach target (end of function)!");
            return new List<Vector3>();
        }

        private List<Vector3> RetracePath(Node startNode, Node endNode)
        {
            ObstacleMap obstacleMap = mapManager.GetObstacleMap();

            List<Vector3> path = new List<Vector3>();
            Node currentNode = endNode;

            while (currentNode != startNode)
            {
                Vector2Int gridPosition = currentNode.worldPosition;

                Vector3 localPosition = obstacleMap.grid.LocalToWorld(obstacleMap.grid.CellToLocalInterpolated(new Vector3Int(gridPosition.x, 0, gridPosition.y)));
                Vector3 worldPosition = mapManager.grid.LocalToWorld(localPosition);

                path.Add(worldPosition);
                currentNode = currentNode.parent;
            }
            path.Reverse();
            return path;
        }
        private static int GetDistance(Vector2Int worldPoisitionA, Vector2Int worldPositionB)
        {
            int distX = Mathf.Abs(Mathf.RoundToInt(worldPoisitionA.x - worldPositionB.x));
            int distY = Mathf.Abs(Mathf.RoundToInt(worldPoisitionA.y - worldPositionB.y));
            //if (distX > distY)
            //    return 14 * distY + 10 * (distX - distY);
            //return 14 * distX + 10 * (distY - distX);
            return Mathf.Max(Mathf.Abs(worldPoisitionA.x - worldPositionB.x), Mathf.Abs(worldPoisitionA.y - worldPositionB.y));

            //return Mathf.RoundToInt(Vector3.Distance(worldPoisitionA, worldPositionB));
        }
        private List<Node> GetNeighbours(Node node, Vector2Int startPos, Vector2Int targetPos)
        {
            List<Node> neighbours = new List<Node>();
            var obstacleMap = mapManager.GetObstacleMap();
            Dictionary<Vector2Int, ObstacleMap.Traversability> mapData = obstacleMap.traversabilityPerCell;
            //foreach (Vector2Int key in mapData.Keys)
            //{
            //Debug.Log(key.ToString());
            //}
            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    if (x == 0 && y == 0)
                        continue;


                    var worldPoint = node.worldPosition + new Vector2Int(x, y);
                    Vector2Int vector2Int = new Vector2Int(Mathf.RoundToInt(worldPoint.x), Mathf.RoundToInt(worldPoint.y));

                    //Debug.Log("checking Neighbour");
                    //Debug.Log(vector2Int);


                    if (mapData.ContainsKey(vector2Int))
                    {
                        //Debug.Log("GridPosition Valid");

                        ObstacleMap.Traversability traversability = mapData[vector2Int];

                        if (traversability == ObstacleMap.Traversability.Free)
                        {
                            Node nNode = new Node(vector2Int, startPos, targetPos );
                            Debug.Log("Added " + vector2Int + " with fCost/hCost : " + nNode.fCost + "/" + nNode.hCost + " as Neighbour to " + node.worldPosition);
                            neighbours.Add(nNode);
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
            
            //Debug.Log(grid_start_pos);
            List<Vector3> my_path = new List<Vector3>();


            // DU ÄNDRADE ALLT TILL VECTOR2INT, NÄR DU ÄNDRADE DET SÅ BLEV FOR LOOPEN INFINITE ELR NÅT

            
            Vector2Int start_vector2Int = new Vector2Int(Mathf.FloorToInt(grid_start_pos.x), Mathf.FloorToInt(grid_start_pos.y));
            Vector2Int goal_vector2Int = new Vector2Int(Mathf.FloorToInt(grid_goal_pos.x), Mathf.FloorToInt(grid_goal_pos.y));

            Debug.Log(start_vector2Int);
            my_path = FindPath(start_vector2Int, goal_vector2Int);
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
            //Debug.Log(grid_start_pos);
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
