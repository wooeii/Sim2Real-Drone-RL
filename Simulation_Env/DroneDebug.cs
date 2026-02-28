using UnityEngine;

public class DroneDebug : MonoBehaviour
{
    public Transform target;
    public Rigidbody parent;
    float t;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        t = Time.time;
    }

    // Update is called once per frame
    void Update()
    {
        if(Time.time - t > 1.0)
        {
            Debug.Log(parent.transform.localPosition);
            Debug.Log(target.localPosition);
            t = Time.time;
        }
    }
}
