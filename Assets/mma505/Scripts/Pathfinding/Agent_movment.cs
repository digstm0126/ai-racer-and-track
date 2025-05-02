namespace mma505
{
    using System;
    using System.Collections.Generic;
    using TMPro;
    using UnityEngine;
    using UnityEditor;
    using System.Net;

    public class Agent_movment : MonoBehaviour
    {

        [Header("debug settings")]
    
        public bool debugRays = false; // debug mode
        public bool RenderLine = false; // render the line
        public bool RenderTargetGizmo = false; // render the target gizmo

        [Header("Ai settings")]

        public float newTargetDis = 5f; // if colided how far the new target will be
        public float resettimer = 5f; // time till the agent resets the target
        private float timer; // timer to reset the target
        public float switchPoint = 1f; // distance to the target position before switching to the next node
        public Vector3 targetPos;  //target position
        public float maxvelocity = 10f; //maximum velocity
        public int offset = 1; // how many nodes ahead the agent will it target
        private Rigidbody rb;


        public enum Algorithm
        {
            BreadthFirstSearch,
            Dijkstra,
            AStar,
            AStar_Euclidian,
            AStar_Manhattan,
            AStar_Diagonal,
            AStar_Octile
        }

        [Header("Algorithm Settings")]


        private Transform start;
        private Transform end;
        private TerrainGraph graph;
        private float gridOffset = 0.5f;
        private List<Node> path = new List<Node>();
        private LineRenderer lr; //line renderer
        private Vector3[] lineVertices; //vertices of the line renderer
        public Algorithm algorithm = Algorithm.BreadthFirstSearch;
      


        void Start()
        {
            rb = GetComponent<Rigidbody>();
            graph = new TerrainGraph();
            end = GameObject.FindWithTag("goal").transform;
            start = transform; // can change, to lazy
            if(RenderLine) lr = GetComponent<LineRenderer>();
            RenderPath();
            targetPos = lineVertices[offset];
            timer = resettimer; 

        }

        private void Update() 
        {

            // set the target position to the desired position
            SetTargetPos(offset);

            // obstacal avoidance
            ObstacleAvoidance();

            // move towards the target position
            MoveToTarget();

            // update the timer
            timer -= Time.deltaTime;
            if (timer <= 0)
            {
                if (Vector3.Distance(transform.position, end.transform.position) <= 0.5f)
                {
                    // reset the target position
                    timer = resettimer;
                    RenderPath();
                    targetPos = lineVertices[offset];
                }

            }
            
        }

        private void ObstacleAvoidance()
        {
            Vector3 direction = targetPos - transform.position;
            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, 5f))
            {
                if (hit.collider.CompareTag("Obstacle") || hit.collider.CompareTag("agent"))
                {
                    if (debugRays) Debug.Log("Obstacle/agent hit front");

                    Vector3 leftDirection = Quaternion.Euler(0, -45, 0) * direction;
                    RaycastHit leftHit;
                    if (Physics.Raycast(transform.position, leftDirection, out leftHit, 5f))
                    {
                        if (leftHit.collider.CompareTag("Obstacle") || leftHit.collider.CompareTag("agent"))
                        {
                            if (debugRays) Debug.Log("Obstacle/agent hit left");

                            Vector3 rightDirection = Quaternion.Euler(0, 45, 0) * direction;
                            RaycastHit rightHit;
                            if (Physics.Raycast(transform.position, rightDirection, out rightHit, 5f))
                            {
                                if (rightHit.collider.CompareTag("Obstacle") || rightHit.collider.CompareTag("agent"))
                                {
                                    if (debugRays) Debug.Log("Obstacle/agent hit right");
                                    Vector3 newDirection = Quaternion.Euler(0, -90, 0) * direction;
                                    targetPos = transform.position + newDirection.normalized * newTargetDis;
                                }
                                else
                                {
                                    Vector3 newDirection = Quaternion.Euler(0, 45, 0) * direction;
                                    targetPos = transform.position + newDirection.normalized * newTargetDis;
                                }
                            }
                        }
                        else
                        {
                            Vector3 newDirection = Quaternion.Euler(0, -45, 0) * direction;
                            targetPos = transform.position + newDirection.normalized * newTargetDis;
                        }
                    }
                }
            }
        }

        private void RenderPath()
        {
            // Start node position
            int startX = (int)start.position.x;
            int startZ = (int)start.position.z;

            Node startNode = graph.grid[startX, startZ];

            // End node position
            int endX = (int)end.position.x;
            int endZ = (int)end.position.z;

            Node endNode = graph.grid[endX, endZ];

            // Get list of nodes making the path based on selected algorithm

            switch (algorithm)
            {
                case Algorithm.BreadthFirstSearch:
                    path = PathAlgorithm.BFS(graph, startNode, endNode);
                    break;
                case Algorithm.Dijkstra:
                    path = PathAlgorithm.Dijkstra(graph, startNode, endNode);
                    break;
                case Algorithm.AStar:
                    path = PathAlgorithm.AStar(graph, startNode, endNode, (int)Algorithm.AStar);
                    break;
                case Algorithm.AStar_Euclidian:
                    path = PathAlgorithm.AStar(graph, startNode, endNode, (int)Algorithm.AStar_Euclidian);
                    break;
                case Algorithm.AStar_Manhattan:
                    path = PathAlgorithm.AStar(graph, startNode, endNode, (int)Algorithm.AStar_Manhattan);
                    break;
                case Algorithm.AStar_Diagonal:
                    path = PathAlgorithm.AStar(graph, startNode, endNode, (int)Algorithm.AStar_Diagonal);
                    break;
                case Algorithm.AStar_Octile:
                    path = PathAlgorithm.AStar(graph, startNode, endNode, (int)Algorithm.AStar_Octile);
                    break;
            }

            // Create the line using an array of vertices based on the nodes in the path
            // Grid offset points to the center of the node cell
            lineVertices = new Vector3[path.Count];
            int index = 0;

            foreach (Node n in path)
            {
                float x = n.nodePosition.X + gridOffset;
                float y = n.nodeHeight + gridOffset;
                float z = n.nodePosition.Y + gridOffset;

                lineVertices[index++] = new Vector3(x, y, z);
            }

            if (RenderLine == true)
            {
                // Render the path
                lr.positionCount = path.Count;
                lr.SetPositions(lineVertices);  
            }
 
            
        }


        public void SetTargetPos(int offset) // sets the target node in path offset aheaad of curnt node
        {
    
            // check if the path is not null
            if (lineVertices != null)
            {
                // check if agent is close to the target position
                if (Vector3.Distance(transform.position, targetPos) < switchPoint)
                {

                    RenderPath();

                    if (offset >= lineVertices.Length)
                    {
                        offset = lineVertices.Length - 1; // clamp the offset to the length of the path
                    }
                    else if (offset < 0)
                    {
                        offset = 0; // clamp the offset to 0
                    }

                    // set the target position to the desired position
                    targetPos = lineVertices[offset];

                    ObstacleAvoidance();
                    
                }
               
            }

        }

        private void MoveToTarget()
        {
            if (rb != null && targetPos != null)
            {
                Vector3 direction = targetPos - transform.position;

                if (direction.magnitude > 0.1f)
                {

                    // Direct velocity modification
                    Vector3 desiredVelocity = direction.normalized * maxvelocity;
                    Vector3 velocityChange = desiredVelocity - rb.linearVelocity;
                    rb.linearVelocity += velocityChange * Time.fixedDeltaTime;
                    
                    //clamp the velocity to maxvelocity
                    rb.linearVelocity = Vector3.ClampMagnitude(rb.linearVelocity, maxvelocity);
                    rb.angularVelocity = Vector3.ClampMagnitude(rb.angularVelocity, 7f);

                }
                else
                {
                    rb.linearVelocity = Vector3.zero;
                    rb.angularVelocity = Vector3.zero;
                }
            }
        }

        private void OnDrawGizmos()
        {
            if (RenderTargetGizmo)
            {
                // Draw the path in the editor
                Gizmos.color = Color.red;
                Gizmos.DrawLine(transform.position, targetPos);
                Gizmos.DrawSphere(targetPos, 0.5f);
            }
        
        }
    }   
}
