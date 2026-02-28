using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using static UnityEngine.EventSystems.StandaloneInputModule;
using YueUltimateDronePhysics;
using JetBrains.Annotations;
public class DroneAgent2: Agent
{
    public Rigidbody rb;

    [SerializeField]
    private Vector3 initPosition;
    [SerializeField]
    private Quaternion initQuaternion;
    [SerializeField]
    private YueDronePhysics dronePhysics;
    [SerializeField]
    private YueInputModule inputModule;

    private bool debug1 = false;
    private bool debug2 = false;

    private Vector3 prevVector;
    private float YawPushed;

    private void Start()
    {
        dronePhysics = GetComponent<YueDronePhysics>();
        inputModule = GetComponent<YueInputModule>();
        rb = GetComponent<Rigidbody>();
        initPosition = rb.transform.position;
        initQuaternion = rb.transform.rotation;
        //Debug.Log("msg1");
    }

    public Transform target;
    public override void OnEpisodeBegin()
    {
        //Debug.Log("msg1");
        //base.OnEpisodeBegin();

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        rb.transform.position = initPosition;
        rb.transform.rotation = initQuaternion;

        prevVector = new Vector3(rb.transform.position.x, 0, rb.transform.position.z);
        YawPushed = 0f;

        target.localPosition = new Vector3(Random.value * 8 - 4, 12.0f, Random.value * 8 - 4);
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        //base.CollectObservations(sensor);
        sensor.AddObservation(target.localPosition);
        sensor.AddObservation(rb.transform.localPosition);
        //sensor.AddObservation(parent.transform.localRotation);
        sensor.AddObservation(rb.transform.localRotation.eulerAngles);

        sensor.AddObservation(rb.GetAccumulatedForce() / rb.mass);
    }

    

    public override void OnActionReceived(ActionBuffers action)
    {
        //float dt = Time.fixedDeltaTime;
        //base.OnActionReceived(actions);
        inputModule.rawLeftHorizontal = action.ContinuousActions[0];   //yaw
        inputModule.rawLeftVertical = action.ContinuousActions[1];     //pitch
        inputModule.rawRightHorizontal = action.ContinuousActions[2];  //roll
        inputModule.rawRightVertical = action.ContinuousActions[3];    //thrust

        YawPushed += action.ContinuousActions[1];

        if (debug1)
            Debug.Log(action.ContinuousActions[0] + " " + action.ContinuousActions[1] + " " + action.ContinuousActions[2] + " " + action.ContinuousActions[3]);

        float distanceToTarget = Vector3.Distance(rb.transform.localPosition, target.localPosition);
        //Debug.Log("distance: " + distanceToTarget);
        //Debug.Log(parent.transform.localPosition);
        //Debug.Log(target.localPosition);
        float rw = -distanceToTarget / 10 + target.localPosition.y - Mathf.Abs(target.localPosition.y - rb.transform.localPosition.y) - (Mathf.Max(Mathf.Abs(YawPushed) - 5f, 0) / 15);
        if (distanceToTarget < 0.5f)
        {
            SetReward(1.0f);
        }

        SetReward(rw);

        if (debug2)
            Debug.Log("rw: " + rw + " 1: " + -distanceToTarget / 10 + " 2: " + (target.transform.position.y - Mathf.Abs(target.transform.position.y - rb.transform.position.y)));

        if (distanceToTarget > 20.0f || !(rb.transform.eulerAngles.x <= 30 || rb.transform.eulerAngles.x >= 330) || rb.transform.position.y <= 0 || Mathf.Max(Mathf.Abs(YawPushed) - 5f, 0) / 15 >= 5.0f) 
        {
            SetReward(-5.0f);
            EndEpisode();
        }
    }
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        //base.Heuristic(actionsOut);
        var continuousAction = actionsOut.ContinuousActions;
        continuousAction[0] = 0.0f;
        continuousAction[1] = 0.0f;
        continuousAction[2] = 0.0f;
        continuousAction[3] = 0.0f;
        if (Input.GetMouseButton(0))
        {
            continuousAction[0] = 1.0f;
        }
    }
}
