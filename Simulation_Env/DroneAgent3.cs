using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using static UnityEngine.EventSystems.StandaloneInputModule;
using YueUltimateDronePhysics;
using JetBrains.Annotations;
public class DroneAgent3: Agent
{
    public Rigidbody rb;
    public DecisionRequester dr;

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
    private float elapsedTime;
    private float prevDistance;

    private void Start()
    {
        dronePhysics = GetComponent<YueDronePhysics>();
        inputModule = GetComponent<YueInputModule>();
        rb = GetComponent<Rigidbody>();
        initPosition = rb.transform.position;
        initQuaternion = rb.transform.rotation;
        //Debug.Log("msg1");
    }

    private void FixedUpdate()
    {
        Debug.Log(elapsedTime);
        if ((int)(elapsedTime += Time.fixedDeltaTime) != (int)elapsedTime)
            Debug.Log("time: " + (elapsedTime += Time.fixedTime));
        elapsedTime += Time.fixedDeltaTime;
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

        dronePhysics.ResetInternals();

        elapsedTime = 0.0f;

        target.localPosition = new Vector3(0, 6.0f, 0);
        prevDistance = Vector3.Distance(rb.transform.localPosition, target.transform.localPosition);
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        //base.CollectObservations(sensor);
        //sensor.AddObservation(target.localPosition);
        //sensor.AddObservation(rb.transform.localPosition);
        //sensor.AddObservation(parent.transform.localRotation);

        sensor.AddObservation(elapsedTime);

        sensor.AddObservation(rb.angularVelocity);

        sensor.AddObservation(rb.GetAccumulatedForce() / rb.mass);
    }

    

    public override void OnActionReceived(ActionBuffers action)
    {
        //float dt = Time.fixedDeltaTime;
        //base.OnActionReceived(actions);
        inputModule.rawLeftHorizontal = 0;   //yaw
        inputModule.rawLeftVertical = action.ContinuousActions[0];     //pitch
        inputModule.rawRightHorizontal = action.ContinuousActions[1];  //roll
        inputModule.rawRightVertical = action.ContinuousActions[2];    //thrust

        if (debug1)
        {
            Debug.Log("0: " + action.ContinuousActions[0] + " 1: " + action.ContinuousActions[1] + " 2: " + action.ContinuousActions[2]);
        }

        float distanceToTarget = Vector3.Distance(rb.transform.localPosition, target.localPosition);
        //Debug.Log(parent.transform.localPosition);
        //Debug.Log(target.localPosition);
        //float rw = 1 - distanceToTarget / 2 + target.localPosition.y - Mathf.Abs(target.localPosition.y - rb.transform.localPosition.y);
        //float rw = -distanceToTarget + target.localPosition.y - Mathf.Abs(target.localPosition.y - rb.transform.localPosition.y);
        SetReward(1f);
        AddReward((prevDistance - distanceToTarget) * 10 / prevDistance);  //향상된 거리 보상

        prevDistance = distanceToTarget;

        AddReward(rb.angularVelocity.magnitude * -0.1f); //너무 돌지 않게 설정

        AddReward(Vector3.Dot(rb.linearVelocity.normalized, (target.position - rb.position).normalized));  //방향에 따른 보상 설정

        if (distanceToTarget < 0.1f)
        {
            AddReward(1.0f);
        }

        if (debug2)
        {
            //Debug.Log("rw: " + rw);
            Debug.Log("distance: " + distanceToTarget);
        }

        if (distanceToTarget > 10.0f || !(rb.transform.eulerAngles.x <= 45 || rb.transform.eulerAngles.x >= 315) || rb.transform.position.y <= 0) 
        {
            SetReward(-10f);
            EpisodeInterrupted();
        }
    }

    /*
    public override void OnActionReceived(ActionBuffers action)
    {
        //float dt = Time.fixedDeltaTime;
        //base.OnActionReceived(actions);
        inputModule.rawLeftHorizontal = 0;   //yaw
        inputModule.rawLeftVertical = action.ContinuousActions[0];     //pitch
        inputModule.rawRightHorizontal = action.ContinuousActions[1];  //roll
        inputModule.rawRightVertical = action.ContinuousActions[2];    //thrust

        if (debug1)
        {
            Debug.Log("0: " + action.ContinuousActions[0] + " 1: " + action.ContinuousActions[1] + " 2: " + action.ContinuousActions[2]);
        }

        float distanceToTarget = Vector3.Distance(rb.transform.localPosition, target.localPosition);
        //Debug.Log(parent.transform.localPosition);
        //Debug.Log(target.localPosition);
        float rw = 3 - distanceToTarget / 2 + target.localPosition.y - Mathf.Abs(target.localPosition.y - rb.transform.localPosition.y);

        SetReward(rw);

        if (distanceToTarget < 0.1f)
        {
            AddReward(1.0f);
        }

        if (debug2)
        {
            Debug.Log("rw: " + rw);
            Debug.Log("distance: " + distanceToTarget);
        }

        if (distanceToTarget > 10.0f || !(rb.transform.eulerAngles.x <= 45 || rb.transform.eulerAngles.x >= 315) || rb.transform.position.y <= 0)
        {
            SetReward(-15f);
            EpisodeInterrupted();
        }
    }
    */
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        //base.Heuristic(actionsOut);
        var continuousAction = actionsOut.ContinuousActions;
        continuousAction[0] = 0.0f;
        continuousAction[1] = 0.0f;
        continuousAction[2] = 0.0f;
        if (Input.GetMouseButton(0))
        {
            continuousAction[0] = 1.0f;
        }
    }
}
