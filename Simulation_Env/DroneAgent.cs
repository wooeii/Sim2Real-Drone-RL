using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
public class DroneAgent: Agent
{
    public Rigidbody rb;
    public Rigidbody parent;
    public Rigidbody motofr;
    public Rigidbody motofl;
    public Rigidbody motorr;
    public Rigidbody motorl;
    public float power;

    [SerializeField]
    private Vector3 initPosition;
    [SerializeField]
    private Quaternion initQuaternion;

    private bool debug1 = false;
    private bool debug2 = false;

    private void Start()
    {
        rb = GetComponent<Rigidbody>();
        initPosition = parent.transform.position;
        initQuaternion = parent.transform.rotation;
        //Debug.Log("msg1");
    }

    public Transform target;
    public override void OnEpisodeBegin()
    {
        //Debug.Log("msg1");
        //base.OnEpisodeBegin();

        rb.linearVelocity = Vector3.zero;
        parent.linearVelocity = Vector3.zero;
        motofl.linearVelocity = Vector3.zero;
        motofr.linearVelocity = Vector3.zero;
        motorl.linearVelocity = Vector3.zero;
        motorr.linearVelocity = Vector3.zero;

        parent.transform.position = initPosition;
        parent.transform.rotation = initQuaternion;

        target.localPosition = new Vector3(Random.value * 8 - 4, 12.0f, Random.value * 8 - 4);
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        //base.CollectObservations(sensor);
        sensor.AddObservation(target.localPosition);
        sensor.AddObservation(parent.transform.localPosition);
        sensor.AddObservation(parent.transform.localRotation);

        sensor.AddObservation(rb.linearVelocity.x);
        sensor.AddObservation(rb.linearVelocity.y);
        sensor.AddObservation(rb.linearVelocity.z);
    }

    

    public override void OnActionReceived(ActionBuffers action)
    {
        //float dt = Time.fixedDeltaTime;
        //base.OnActionReceived(actions);
        motofl.AddRelativeForce(Vector3.forward * (action.ContinuousActions[0] + 0.5f) * power);
        motofr.AddRelativeForce(Vector3.forward * (action.ContinuousActions[1] + 0.5f) * power);
        motorl.AddRelativeForce(Vector3.forward * (action.ContinuousActions[2] + 0.5f) * power);
        motorr.AddRelativeForce(Vector3.forward * (action.ContinuousActions[3] + 0.5f) * power);
        if(debug1)
            Debug.Log(action.ContinuousActions[0] + " " + action.ContinuousActions[1] + " " + action.ContinuousActions[2] + " " + action.ContinuousActions[3]);

        float distanceToTarget = Vector3.Distance(parent.transform.localPosition, target.localPosition);
        //Debug.Log("distance: " + distanceToTarget);
        //Debug.Log(parent.transform.localPosition);
        //Debug.Log(target.localPosition);
        float rw = -distanceToTarget / 10 + target.localPosition.y - Mathf.Abs(target.localPosition.y - parent.transform.localPosition.y);
        if (distanceToTarget < 0.5f)
        {
            SetReward(1.0f);
        }
        SetReward(rw);
        if (debug2)
            Debug.Log("rw: " + rw + " 1: " + -distanceToTarget / 10 + " 2: " + (target.transform.position.y - Mathf.Abs(target.transform.position.y - parent.transform.position.y)));
        else if (distanceToTarget > 20.0f || rb.transform.eulerAngles.x < 225 || rb.transform.eulerAngles.x > 315)
        {
            SetReward(-1.0f);
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
            continuousAction[1] = 1.0f;
            continuousAction[2] = 1.0f;
            continuousAction[3] = 1.0f;
        }
    }
}
