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
            public Vector3 worldPosition;
            public Node parent;
            public int gCost;
            public int hCost;
            public int fCost { get { return gCost + hCost; } }

            public Node(Vector3 _worldPos)
            {
                worldPosition = _worldPos;
            }
        }
        private List<Vector3> FindPath(Vector3 startWorldPos, Vector3 targetWorldPos)
        {
            Node startNode = new Node(startWorldPos);
            Node targetNode = new Node(targetWorldPos);

            List<Node> openSet = new List<Node>();
            HashSet<Node> closedSet = new HashSet<Node>();
            openSet.Add(startNode);

            while (0)
            {
                Node currentNode = openSet[0];
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
                    return RetracePath(startNode, targetNode);
                }

                foreach (Node neighbour in GetNeighbours(currentNode))
                {
                    if (closedSet.Contains(neighbour))
                    {
                        continue;
                    }

                    int newCostToNeighbour = currentNode.gCost + GetDistance(currentNode, neighbour);
                    if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour))
                    {
                        neighbour.gCost = newCostToNeighbour;
                        neighbour.hCost = GetDistance(neighbour, targetNode);
                        neighbour.parent = currentNode;

                        if (!openSet.Contains(neighbour))
                            openSet.Add(neighbour);
                    }
                }
            }

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
        private int GetDistance(Node nodeA, Node nodeB)
        {
            int distX = Mathf.Abs(Mathf.RoundToInt(nodeA.worldPosition.x - nodeB.worldPosition.x));
            int distZ = Mathf.Abs(Mathf.RoundToInt(nodeA.worldPosition.z - nodeB.worldPosition.z));
            if (distX > distZ)
                return 14 * distZ + 10 * (distX - distZ);
            return 14 * distX + 10 * (distZ - distX);
        }
        private List<Node> GetNeighbours(Node node)
        {
            List<Node> neighbours = new List<Node>();

            for (int x = -1; x <= 1; x++)
            {
                for (int z = -1; z <= 1; z++)
                {
                    if (x == 0 && z == 0)
                        continue;

                    Vector3 worldPoint = node.worldPosition + new Vector3(x, 0, z);
                    var obstacleMap = mapManager.GetObstacleMap();
                    if (obstacleMap.IsLocalPointTraversable(worldPoint) == ObstacleMap.Traversability.Free)
                    {
                        neighbours.Add(new Node(worldPoint));
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

            Vector3 start_pos = mapManager.localStartPosition;
            Vector3 goal_pos = mapManager.localGoalPosition;

            List<Vector3> my_path = new List<Vector3>();

            path = FindPath(start_pos, goal_pos);

            // Plot your path to see if it makes sense
            // Note that path can only be seen in "Scene" window, not "Game" window
            Vector3 old_wp = start_pos;
            foreach (var wp in my_path)
            {
                Debug.DrawLine(mapManager.grid.LocalToWorld(old_wp), mapManager.grid.LocalToWorld(wp), Color.white, 1000f);
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
