using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using static UnityEngine.EventSystems.StandaloneInputModule;
using YueUltimateDronePhysics;
using JetBrains.Annotations;
public class DroneAgent4 : Agent
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

        elapsedTime = 0.0f;

        target.localPosition = new Vector3(0, 6.0f, 0);

        prevVector = target.position - rb.position;

        dronePhysics.ResetInternals();
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
        // 드론 제어 입력 처리
        inputModule.rawLeftHorizontal = 0; // yaw
        inputModule.rawLeftVertical = action.ContinuousActions[0]; // pitch
        inputModule.rawRightHorizontal = action.ContinuousActions[1]; // roll
        inputModule.rawRightVertical = action.ContinuousActions[2]; // thrust

        // 목표와 거리 계산
        float curDistance = Vector3.Distance(rb.position, target.position);
        float prevDistance = prevVector.magnitude;

        // 거리 변화 보상 (가까워질수록 +보상)
        float deltaDistReward = prevDistance - curDistance;
        AddReward(deltaDistReward * 1f);

        // 수직 정렬 보상 (목표 높이와의 차이 줄이기)
        float heightError = Mathf.Abs(rb.position.y - target.position.y);
        AddReward(-heightError * 0.05f);

        // 자세 안정화 보상 (회전이 심하면 패널티)
        float anglePenalty = rb.angularVelocity.magnitude;
        AddReward(-anglePenalty * 0.01f);

        // 가속도 페널티 (급작스런 힘 억제)
        float accelPenalty = (rb.GetAccumulatedForce() / rb.mass).magnitude;
        AddReward(-accelPenalty * 0.01f);

        // 시간 지남에 따른 소량 감점
        AddReward(-0.001f);

        // 목표에 거의 도달하면 추가 보상
        if (curDistance < 0.1f && heightError < 0.1f)
        {
            AddReward(5.0f);
        }

        // 실패 조건 (멀어짐, 바닥, 과도한 회전)
        if (curDistance > 10.0f || anglePenalty > 5f || rb.position.y <= 0)
        {
            AddReward(-10f);
            EndEpisode(); // EpisodeInterrupted() → EndEpisode()로 변경 권장
        }

        // 다음 비교를 위한 이전 벡터 갱신
        prevVector = target.position - rb.position;
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
